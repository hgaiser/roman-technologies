/*
 * SoundPlayer.cpp
 *
 *  Created on: 2012-01-24
 *      Author: wilson
 */

#include <audio_processing/SoundPlayer.h>

/// Maps incoming Emotion to a file path for playback
static std::map<u_int8_t, std::string> EmotionToPath;

bool SoundPlayer::playSound(std::string Path)
{
	if (!mBuffer.LoadFromFile(Path.c_str()))
		return false;

	// Create a sound instance and play it
	sf::Sound Sound(mBuffer);
	Sound.Play();
	sf::Sleep(0.4f);

	// Loop while the sound is playing
	while (Sound.GetStatus() == sf::Sound::Playing)
	{
		// Display the playing position
		std::cout << "\rPlaying... " << std::fixed << std::setprecision(2) << Sound.GetPlayingOffset() << " sec";

		// Leave some CPU time for other threads
		sf::Sleep(0.1f);
	}

	return true;
}

void SoundPlayer::playCB(const std_msgs::UInt8& msg)
{
	if(playSound(EmotionToPath[msg.data]))
		ROS_INFO("Played sound successfully");
	else
		ROS_ERROR("Could not find wav file");
}

/**
 * Initialise SoundPlayer
 */
void SoundPlayer::init()
{
	//Initialise Subscribers
	mSoundSub = mNodeHandle.subscribe("/cmd_sound", 1, &SoundPlayer::playCB, this);

	ROS_INFO("Initialising complete");
}


int main( int argc, char* argv[])
{
	ros::init(argc, argv, "SoundPlayer");

	if (argc != 4)
	{
		ROS_ERROR("Invalid input arguments.");
		return 0;
	}

	SoundPlayer soundPlayer;
	soundPlayer.init();

	EmotionToPath[HAPPY] 	 = argv[1];
	EmotionToPath[SAD]   	 = argv[2];
	EmotionToPath[SURPRISED] = argv[3];

	ros::spin();

	return EXIT_SUCCESS;
}