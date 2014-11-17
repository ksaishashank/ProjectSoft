#ifndef _MOVIE_PLAYER_H
#define _MOVIE_PLAYER_H

#include "scene/scene.h"
#include "utility/archive.h"

class MoviePlayer
{
public:
	MoviePlayer();
	MoviePlayer(DecoScene* scene);
	~MoviePlayer();
	void SetScene(DecoScene* scene);
	void SetVersion(double version);
	void OpenFile(const string& movieFileName);
	void Seek(int ithFrame);
	void Seek(float time);
	int Tell() const;
	void Play();
	void PlayIthFrame(int ithFrame);
	void Pause();
	void Next();
	void Previous();
	bool IsPlaying() const;
    void SaveGeometry(const string& filename);
    void SaveForce(const string& filename);
	void SaveMuscle(const string& filename);
private:
	void decrementFrameCounter(bool isLoop = true);
	void incrementFrameCounter(bool isLoop = true);
	DecoScene* mScene;
	double mTimeStep;
	int mIthFrame;
	DecoArchive mMovieFile;
	bool mIsPaused;
	int mNumFrames;
	double mVersion;
};

#endif