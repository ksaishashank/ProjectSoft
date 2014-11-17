#include "MovieRecorder.h"
#include "utility/timer.h"
//#include <boost/filesystem/operations.hpp>
//namespace bf = boost::filesystem;

MovieRecorder::MovieRecorder() : mIthFrame(0), mIsInitialized(false)
{
    

}
MovieRecorder::MovieRecorder(DecoScene* scene) : mIthFrame(0), mIsInitialized(false)
{
    SetScene(scene);
}
MovieRecorder::~MovieRecorder()
{

}

void MovieRecorder::SetScene(DecoScene* scene)
{
    mScene = scene;
}

void MovieRecorder::Initialize()
{
	if (!mIsInitialized)
	{
		constructNewMovieFile();
		if (PLAYER_VERSION >= 0.4)
		{
			mMovieFile << PLAYER_VERSION;
		}
		mMovieFile << mScene->GetTimeStep();
		vector<DecoSceneObject*> allObjs;
		mScene->GetAllSceneObjects(allObjs);
		int numObjs = static_cast<int>(allObjs.size());
		mMovieFile << numObjs;
		for (int i = 0; i < numObjs; ++i)
		{
			mMovieFile << allObjs[i]->GetName();
			int type = (int)(allObjs[i]->GetType());
			mMovieFile << type;
			allObjs[i]->RecordMovie(mMovieFile, PLAYER_VERSION); //first the initial pose
		}
		mMovieFile.Flush();
		mIsInitialized = true;
	}
}
void MovieRecorder::Record()
{
	//bf::path videoFilePath(mFilename);
	//bf::path absolutePath = bf::complete(videoFilePath);
	//
	//LOG(INFO) << mIthFrame << "th frame is recorded in " << absolutePath;

    mMovieFile << mIthFrame;
    mScene->RecordMovie(mMovieFile, PLAYER_VERSION);
	mMovieFile.Flush();
	LOG(INFO) << "Finish recording " << mIthFrame << "th frame.";
	mIthFrame++;
}

void MovieRecorder::constructNewMovieFile()
{
    time_t rawTime;
	time(&rawTime);
	struct tm* timeinfo;
	timeinfo = localtime(&rawTime);
	char timeStrBuff[256];
    strftime(timeStrBuff, 256, "%b%d-%H-%M-%S", timeinfo);

	stringstream ss;
	ss << "../../../movie/movie";
	ss << timeStrBuff;
    ss << ".jie";
	mFilename = ss.str();
    mMovieFile.Create(mFilename, AT_Write);    
}