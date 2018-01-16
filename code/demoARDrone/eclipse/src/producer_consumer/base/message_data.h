/*
 * message_data.h
 *
 *  Created on: Aug 30, 2013
 *      Author: truongnt
 */

#ifndef MESSAGE_DATA_H_
#define MESSAGE_DATA_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//https://groups.google.com/forum/#!topic/boost-list/15nKgxGFAeQ
#include <boost/date_time/posix_time/time_serialize.hpp>

enum Command {
	NoCommand,
	ClientSendInfoToServer,
	CloseConnection,
	//remote Drone

	Escape,

	TakeOff,
	Landing,

	Engage,
	DisEngage,

	MoveForward,
	MoveBackward,

	MoveUp,
	MoveDown,

	MoveLeft,
	MoveRight,

	RotateLeft,
	RotateRight,

	//for engaged mode
	AutoMoveForward,
	AutoMoveBackward,

	AutoMoveUp,
	AutoMoveDown,

	AutoMoveLeft,
	AutoMoveRight,

	AutoRotate,
};
class MessageData {
	//only need when we want to access private variables
	//friend class boost::serialization::access;
public:
	cv::Mat mImg;
	Command mCommand;
	int mLapNo;
	unsigned int mversion;
	boost::posix_time::ptime mTimeStamp;

	int mCommandIndex;

	MessageData(const MessageData &cSource)
	{
		mImg = cSource.mImg.clone();
		mCommand = cSource.mCommand;
		mLapNo = cSource.mLapNo;
		mTimeStamp = cSource.mTimeStamp;
		mCommandIndex = cSource.mCommandIndex;
	}

	MessageData() {
		mCommandIndex = -1;
		mCommand = Command::NoCommand;
	}

	template <class Archive>
	void serialize(
			Archive& ar,
			unsigned int version
	)
	{
		//std::cout<<"serialization MessageData version="<<version<<std::endl;
		//printing("serialization MessageData version=" + utilities::NumberToString(version));
		ar & mImg;
		ar & mLapNo;
		ar & mCommand;
		ar & mTimeStamp;

		ar & mCommandIndex;

		mversion = version;
	}

	bool isCommandMessage(const Command& cm) {
		if (this->mCommand == cm) {
			return true;
		}
		return false;
	}

	static MessageData createMessage(const Command& cm) {
		MessageData msg2;
		msg2.mCommand = cm;
		msg2.mTimeStamp = boost::posix_time::microsec_clock::local_time();
		return msg2;
	}

	static MessageData createMessageCommand(int index, const Command& cm) {
			MessageData msg2;
			msg2.mCommand = cm;
			msg2.mTimeStamp = boost::posix_time::microsec_clock::local_time();
			msg2.mCommandIndex = index;
			return msg2;
		}
};

#endif /* MESSAGE_DATA_H_ */
