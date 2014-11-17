#ifndef _MOVIERECORDER_H
#define _MOVIERECORDER_H

#include "scene/scene.h"
#include "utility/archive.h"

#define PLAYER_VERSION 0.5

class MovieRecorder
{
public:
    MovieRecorder();
    MovieRecorder(DecoScene* scene);
    ~MovieRecorder();
    void SetScene(DecoScene* scene);
    void Record();
	void Initialize();
private:
    void constructNewMovieFile();
    DecoScene* mScene;
    DecoArchive mMovieFile;
	int mIthFrame;
	string mFilename;
	bool mIsInitialized;
};

#endif