/*
 * sender_receiver.hpp
 *
 *  Created on: Aug 13, 2013
 *      Author: truongnt
 */

#ifndef SENDER_RECEIVER_HPP_
#define SENDER_RECEIVER_HPP_
//netstat --listen

#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include "base/base_services.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "boost/thread/mutex.hpp"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using boost::asio::ip::tcp;

enum ConnectionState {
	Step1,
	Step2,
	Step3,
	Step4,
	Step5,
	Step6,
	ConnectioEstablished,
	activeConnection,
	inactive
};

class session_server_base
{
public:
	session_server_base(boost::asio::io_service& io_service)
	: socket_(io_service)
	{
	}

	virtual ~session_server_base() {

	}

	tcp::socket& socket()
	{
		return socket_;
	}

	ConnectionState mconnectionState;
	mutable boost::mutex mutex_connectionState;
	bool checkConnectionState(const ConnectionState& currentState) {
		boost::mutex::scoped_lock lock(mutex_connectionState);
		if (this->mconnectionState == currentState) return true;
		return false;
	}

	void changeConnectionState(const ConnectionState& currentState) {
		boost::mutex::scoped_lock lock(mutex_connectionState);
		this->mconnectionState = currentState;
	}

	virtual void process_receiver(MessageData & msg) {
		printing_with_color("session_server_base::process_receiver" + utilities::NumberToString(msg.mLapNo), utilities::Color::cyan);
	}
	void asyn_getMessage()
	{
		//start to receive something
		socket_.async_read_some(boost::asio::buffer(&header_, sizeof(header_)),
				boost::bind(&session_server_base::handle_read_header, this,
						boost::asio::placeholders::error,
						boost::asio::placeholders::bytes_transferred));
	}

	void handle_read_header(const boost::system::error_code& error,
			size_t bytes_transferred)
	{
		if (!error)
		{
			printing("Body message: " + utilities::NumberToString(header_) + " bytes, transfered: " + utilities::NumberToString(bytes_transferred));
			if (sizeof(header_) == 4) // in case 32 bits, add more 4 bits to align with 64bits system
			{
				//printing("read more 4 bytes" );
				unsigned int a;
				boost::system::error_code error2;
				socket_.read_some(boost::asio::buffer( &a, sizeof(a) ), error2);
			}

			//--------------------------------------------------------------------------------------
			// read body
			remain_ = header_;

			socket_.async_read_some(streambuf_.prepare(remain_),
					boost::bind(&session_server_base::handle_read_body, this,
							boost::asio::placeholders::error,
							boost::asio::placeholders::bytes_transferred));
		}
		else
		{
			delete this;
		}
	}

	void handle_read_body(const boost::system::error_code& error,
			size_t bytes_transferred)
	{
		if (!error)
		{
			if (bytes_transferred > 0) {
				remain_ -= bytes_transferred;
				//http://www.boost.org/doc/libs/1_46_1/doc/html/boost_asio/reference/basic_streambuf.html
				//Move characters from the output sequence to the input sequence.
				streambuf_.commit(bytes_transferred);
				if (remain_ > 0) {
					socket_.async_read_some(streambuf_.prepare(remain_),
							boost::bind(&session_server_base::handle_read_body, this,
									boost::asio::placeholders::error,
									boost::asio::placeholders::bytes_transferred));
				}
				else {
					printing_with_color("Get a message successfully!", utilities::Color::green);
					// deserialize
					std::istream is( &streambuf_ );
					boost::archive::text_iarchive ar( is );

					MessageData msg;
					ar >> msg;
					this->process_receiver(msg);
				}
			}
		}
		else
		{
			delete this;
		}
	}

private:
	tcp::socket socket_;

	//for header
	size_t header_;

	//for body
	boost::asio::streambuf streambuf_;
	size_t remain_;
};

class session_server1: public session_server_base {
public:
	session_server1(boost::asio::io_service& io_service)
	: session_server_base(io_service)
	{
	}

	virtual void process_receiver(MessageData & msg) override {
		//after finish asyn_getMessage, call this function for process that Message
		//printing("msg command: " + utilities::NumberToString(msg.mCommand));
		if (msg.isCommandMessage(Command::ClientSendInfoToServer)) {
			this->changeConnectionState(ConnectionState::activeConnection);
			this->asyn_getMessage();
			return;
		}
		if (this->checkConnectionState(ConnectionState::activeConnection)) {
			printing("activeConnection");

			GloQueueData.push(msg);
			this->asyn_getMessage();
			return;
			//			if (msg.isCommandMessage(Command::CloseConnection)) {
			//				this->setCloseConnection(true);
			//				printing("Receiver - Close connection!");
			//				break;
			//			}
		}
	}
};


class session_server2: public session_server_base {
public:
	session_server2(boost::asio::io_service& io_service)
	: session_server_base(io_service)
	{
	}
	virtual void process_receiver(MessageData & msg) override {
		//after finish asyn_getMessage, call this function for process that Message
		//printing("msg command: " + utilities::NumberToString(msg.mCommand));
		if (msg.isCommandMessage(Command::ClientSendInfoToServer)) {
			this->changeConnectionState(ConnectionState::activeConnection);
			this->asyn_getMessage();
			return;
		}
		if (this->checkConnectionState(ConnectionState::activeConnection)) {
			//printing("activeConnection");
			printing("Message command: index: " + utilities::NumberToString(msg.mCommandIndex) + " command: "  + utilities::NumberToString(msg.mCommand));
			//GloQueueData.push(msg);
			GloQueueCommand.push(msg);
			this->asyn_getMessage();
			return;

		}
	}
};
//create this class in DroneConsumer and DroneProducer
class server
{
public:
	server(boost::asio::io_service& io_service, short port, int lapNo)
	: io_service_(io_service),
	  acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
	{
		printing("LISTENING port: " + utilities::NumberToString(port));
		session_server_base* new_session;
		if (lapNo == 1) {
			new_session = new session_server1(io_service_);
		}
		else {
			new_session = new session_server2(io_service_);
		}
		acceptor_.async_accept(new_session->socket(),
				boost::bind(&server::handle_accept, this, new_session,
						boost::asio::placeholders::error));
	}

	void handle_accept(session_server_base* new_session,
			const boost::system::error_code& error)
	{
		if (!error)
		{
			new_session->asyn_getMessage();
			new_session = new session_server_base(io_service_);
			acceptor_.async_accept(new_session->socket(),
					boost::bind(&server::handle_accept, this, new_session,
							boost::asio::placeholders::error));
		}
		else
		{
			//delete new_session;
			//show the error, continue listening util have the signal of stopping
			new_session = new session_server_base(io_service_);
			acceptor_.async_accept(new_session->socket(),
					boost::bind(&server::handle_accept, this, new_session,
							boost::asio::placeholders::error));
		}
	}

private:
	boost::asio::io_service& io_service_;
	tcp::acceptor acceptor_;
};


//---------------------------------------------------------------------------------------------------------
//firstly, build up a sender in here to replace for the old one
class client
{
private:
	boost::asio::io_service& io_service_;
	tcp::socket socket_;
	std::string hostDst_, portDst_;

public:
	bool IsProcessingMessage_;
	mutable boost::mutex mt_protectIsProcessingMessage;
	bool getIsProcessingMessage() {
		boost::mutex::scoped_lock lock(mt_protectIsProcessingMessage);
		return IsProcessingMessage_;
	}
	void setIsProcessingMessage(bool b) {
		boost::mutex::scoped_lock lock(mt_protectIsProcessingMessage);
		printing("setIsProcessingMessage" + utilities::NumberToString(b));
		IsProcessingMessage_ = b;
	}

public:
	client(boost::asio::io_service& io_service,
			std::string hostDst, std::string portDst)
	: io_service_(io_service),
	  socket_(io_service),
	  mt_protectIsProcessingMessage()
	{
		this->changeConnectionState(ConnectionState::Step2);
		hostDst_ = hostDst;
		portDst_ = portDst;


		tcp::resolver resolver(io_service_);
		tcp::resolver::query query(hostDst_, portDst_);
		tcp::resolver::iterator iterator = resolver.resolve(query);

		tcp::endpoint endpoint = *iterator;
		socket_.async_connect(endpoint,
				boost::bind(&client::handle_connect, this,
						boost::asio::placeholders::error, ++iterator));

		setIsProcessingMessage(false);
	}
	virtual ~client() {

	}

	ConnectionState mconnectionState;
	mutable boost::mutex mutex_connectionState;
	bool checkConnectionState(const ConnectionState& currentState) {
		boost::mutex::scoped_lock lock(mutex_connectionState);
		if (this->mconnectionState == currentState) return true;
		return false;
	}

	void changeConnectionState(const ConnectionState& currentState) {
		boost::mutex::scoped_lock lock(mutex_connectionState);
		this->mconnectionState = currentState;
	}


protected:
	void handle_connect(const boost::system::error_code& error,
			tcp::resolver::iterator endpoint_iterator)
	{
		if (!error)
		{
			printing("Connect to server successfully!");
			::setSystemState(SystemState::activeState);
			//this->process_sender();
			setIsProcessingMessage(true);
		}
		else if (endpoint_iterator != tcp::resolver::iterator())
		{
			socket_.close();
			tcp::endpoint endpoint = *endpoint_iterator;
			socket_.async_connect(endpoint,
					boost::bind(&client::handle_connect, this,
							boost::asio::placeholders::error, ++endpoint_iterator));
		}
		else {
			printing("Can't connect! Re-connect");
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));

			tcp::resolver resolver(io_service_);
			tcp::resolver::query query(hostDst_, portDst_);
			tcp::resolver::iterator iterator = resolver.resolve(query);

			tcp::endpoint endpoint = *iterator;
			socket_.async_connect(endpoint,
					boost::bind(&client::handle_connect, this,
							boost::asio::placeholders::error, ++iterator));
		}
	}
public:
	void process_sender() {
		printing_with_color("client::process_sender", utilities::Color::cyan);
		MessageData msg;

		if (this->checkConnectionState(ConnectionState::Step2)) {
			printing("Step 2");

			msg = MessageData::createMessage(Command::ClientSendInfoToServer);
			this->asyn_sendMessage(msg);

			this->changeConnectionState(ConnectionState::activeConnection);//back to step 3 and wait for comming connection
			return;
		}

		if (this->checkConnectionState(ConnectionState::activeConnection)) {
			printing("activeConnection");
			GloQueueData.wait_and_pop(msg);

			this->asyn_sendMessage(msg);
		}

	}
protected:
	boost::asio::streambuf* streambuf_ptr_;
	std::ostream* os_ptr;
	boost::archive::text_oarchive* ar_ptr;
	size_t header_;
	void asyn_sendMessage(MessageData & msg)
	{
		//init variables.
		//		if (streambuf_ptr_) delete streambuf_ptr_;
		//		if (os_ptr) delete os_ptr;
		//		if (ar_ptr) delete ar_ptr;
		streambuf_ptr_ = new boost::asio::streambuf();
		os_ptr = new std::ostream(streambuf_ptr_);
		ar_ptr = new boost::archive::text_oarchive(*os_ptr);

		*ar_ptr << msg;

		size_t header_ = streambuf_ptr_->size();
		printing("body is " + utilities::NumberToString(header_) + " bytes" );

		// send header_ and buffer using scatter
		std::vector<boost::asio::const_buffer> buffers_;
		buffers_.push_back( boost::asio::buffer(&header_, sizeof(header_)) );
		//--------------------------------------------------------------------------------------
		if (sizeof(header_) == 4) // in case 32 bits, add more 4 bits to align with 64bits system
		{
			//printing("write more 4 bytes" );
			unsigned int a = 0;
			buffers_.push_back(boost::asio::buffer(&a, sizeof(a)));
		}
		//--------------------------------------------------------------------------------------
		buffers_.push_back( streambuf_ptr_->data() );

		boost::asio::async_write(socket_,
				buffers_,
				boost::bind(&client::handle_write, this,
						boost::asio::placeholders::error));


	}

	void handle_write(const boost::system::error_code& error)
	{
		if (!error)
		{
			//printing("wrote " + utilities::NumberToString(rc_) + " bytes" );
			//streambuf_.consume(header_);
			printing_with_color("sent a message successfully!", utilities::Color::magenta);

			//process_sender();
			setIsProcessingMessage(true);
		}
		else
		{
			printing("[ERROR] handle_write: " + error.message());
			do_close();
		}
	}

	void do_close()
	{
		socket_.close();
	}
};


class client1 :public client {
public:
	client1(boost::asio::io_service& io_service, std::string hostDst, std::string portDst)
	: client(io_service, hostDst, portDst)
	{
		CommandIndex_ = 0;
	}
public:
	void process_sender();
private:
	int CommandIndex_;
};

class client2 :public client {
public:
	client2(boost::asio::io_service& io_service, std::string hostDst, std::string portDst)
	: client(io_service, hostDst, portDst)
	{

	}
public:
	void process_sender();
};

#endif /* SENDER_RECEIVER_HPP_ */

