/*
 * sender_receiver.cpp
 *
 *  Created on: Aug 13, 2013
 *      Author: truongnt
 */

#include "sender_receiver.h"

//------------------------------------------------------------------------
//client1
void client1::process_sender() {
		while (true) {
			//printing("client1::process_sender");
			if (!getIsProcessingMessage()) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(250));
				continue;
			}

			MessageData msg;

			if (this->checkConnectionState(ConnectionState::Step2)) {
				printing("Step 2");

				msg = MessageData::createMessage(Command::ClientSendInfoToServer);
				this->asyn_sendMessage(msg);

				this->changeConnectionState(ConnectionState::activeConnection);//back to step 3 and wait for comming connection
				setIsProcessingMessage(false);
			}

			if (this->checkConnectionState(ConnectionState::activeConnection)) {
				printing("activeConnection");

				while (true) {
					GloQueueCommand.wait_and_pop(msg);
					CommandIndex_++;
					msg.mCommandIndex = CommandIndex_;

					this->asyn_sendMessage(msg);
				}
				setIsProcessingMessage(false);
			}
		}
	}

//------------------------------------------------------------------------
//client2
void client2::process_sender() {
		while (true) {
			printing("client2::process_sender");
			if (!getIsProcessingMessage()) {
				boost::this_thread::sleep(boost::posix_time::milliseconds(250));
				continue;
			}

			MessageData msg;

			if (this->checkConnectionState(ConnectionState::Step2)) {
				printing("Step 2");

				msg = MessageData::createMessage(Command::ClientSendInfoToServer);
				this->asyn_sendMessage(msg);

				this->changeConnectionState(ConnectionState::activeConnection);//back to step 3 and wait for comming connection
				setIsProcessingMessage(false);
			}

			if (this->checkConnectionState(ConnectionState::activeConnection)) {
				printing("activeConnection");
				GloQueueData.wait_and_pop(msg);
				this->asyn_sendMessage(msg);
				setIsProcessingMessage(false);
			}

			//setIsProcessingMessage(false);
		}
	}
