#include "MoviePlayer.h"
#include "ForceRecorder.h"
#include "ActiveSoftBody.h"

MoviePlayer::MoviePlayer() : mTimeStep(0.033), mIthFrame(-1), mIsPaused(true), mVersion(0.0)
{

}
MoviePlayer::MoviePlayer(DecoScene* scene) : mTimeStep(0.033), mIthFrame(-1), mIsPaused(true), mVersion(0.0)
{
	SetScene(scene);
}
MoviePlayer::~MoviePlayer()
{

}

void MoviePlayer::SetVersion(double version)
{
	mVersion = version;
}
void MoviePlayer::SetScene(DecoScene* scene)
{
	mScene = scene;
}
void MoviePlayer::OpenFile(const string& movieFileName)
{
	string realMovieFileName = movieFileName;
	if (realMovieFileName.find("../../../movie/") == string::npos)
		realMovieFileName = "../../../movie/" + movieFileName;
	if (!mMovieFile.Create(realMovieFileName, AT_Read))
		LOG(FATAL) << "Cannot open movie file " + realMovieFileName;
	if (mVersion >= 0.4)
		mMovieFile >> mVersion;
	double timeStep;
	mMovieFile >> timeStep;
	mScene->SetTimeStep(timeStep);
	int numObjs;
	mMovieFile >> numObjs;

	for (int i = 0; i < numObjs; ++i)
	{
		string objName;
		mMovieFile >> objName;
		int objType;
		mMovieFile >> objType;
		SceneObjectType type = (SceneObjectType) objType;
		if (type == SoftT)
		{
			DecoSoftObject* softObj = new ActiveSoftBody();
			BOOL bSuccess = softObj->CreateSceneObjectFromMesh(objName, timeStep);
			if (!bSuccess)
				LOG(FATAL) << "Construction of " << objName << " failed.";
			softObj->SetInitialPoseForMovie(mMovieFile, mVersion);
			mScene->AddSceneObject(softObj);
		}
	}
	mNumFrames = 0;
	vector<DecoSceneObject*> allObjs;
	mScene->GetAllSceneObjects(allObjs);
	while (!mMovieFile.IsEOF())
	{
		int ithFrame;
		mMovieFile >> ithFrame;
		for (int i = 0; i < numObjs; ++i)
		{
			allObjs[i]->CacheMovie(mMovieFile, mVersion);
		}
		int pos = mMovieFile.Tell();
		int fileSize = mMovieFile.FileSize();
		mNumFrames++;
	}

}

bool MoviePlayer::IsPlaying() const
{
	return (!mIsPaused && mIthFrame >= 0);
}

void MoviePlayer::Next()
{
	incrementFrameCounter();
	PlayIthFrame(mIthFrame);
}

void MoviePlayer::Previous()
{
	decrementFrameCounter();
	PlayIthFrame(mIthFrame);
}
void MoviePlayer::Seek(int ithFrame)
{
	mIthFrame = ithFrame;
    PlayIthFrame(mIthFrame);
}
void MoviePlayer::Seek(float time)
{
	int ithFrame = static_cast<int>(time / mTimeStep);
	Seek(ithFrame);

}
int MoviePlayer::Tell() const
{
	return mIthFrame;
}

void MoviePlayer::SaveMuscle(const string& filename)
{
	if (mVersion < 0.5)
	{
		LOG(WARNING) << "SaveMuscle only supported after version 0.5.";
		return;
	}
	vector<DecoSceneObject*> allObjs;
	mScene->GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	string dir = "../../../obj/";
	char newFileName[256];
	string fileName = CleanPath(filename);

	for (int ithFrame = 0; ithFrame < mNumFrames; ++ithFrame)
	{
		PlayIthFrame(ithFrame);
		for (int ithObj = 0; ithObj < numObjs; ++ithObj)
		{
			if (strstr(allObjs[ithObj]->GetName().c_str(), "ground") != NULL)
				continue;
			string meshName = CleanPath(allObjs[ithObj]->GetName());
			memset(newFileName, 0, 256 * sizeof(char));
			sprintf_s(newFileName, "%s%s%s%d.fiber", dir.c_str(), fileName.c_str(), meshName.c_str(), ithFrame);
			ofstream out(newFileName);

			ActiveSoftBody* softBody = static_cast<ActiveSoftBody*>(allObjs[ithObj]);
			int numFibers = softBody->GetNumMuscleFibers();
			out << numFibers << " " << endl;
			for (int i = 0; i < numFibers; ++i)
			{
				const MuscleFiber& fiber = softBody->GetMuscleFiber(i);
				const vector<Vector3d>& pos = fiber.GetPositions();
				int numVertices = static_cast<int>(pos.size());
				out << numVertices << " ";
				for (int i = 0; i < numVertices; ++i)
				{
					out << pos[i][0] << " " << pos[i][1] << " " << pos[i][2] << " ";
				}
				out << endl;
			}
		}
		LOG(INFO) << "Finish Dumping " << ithFrame << "th frame.";
	}
}
void MoviePlayer::SaveForce(const string& filename)
{
    vector<DecoSceneObject*> allObjs;
    mScene->GetAllSceneObjects(allObjs);
    int numObjs = static_cast<int>(allObjs.size());
    string dir = "../../../obj/";
    char newFileName[256];
    string fileName = CleanPath(filename);

    for (int ithFrame = 0; ithFrame < mNumFrames; ++ithFrame)
    {
        PlayIthFrame(ithFrame);
        for (int ithObj = 0; ithObj < numObjs; ++ithObj)
        {
            if (strstr(allObjs[ithObj]->GetName().c_str(), "ground") != NULL)
                continue;
            string meshName = CleanPath(allObjs[ithObj]->GetName());
            memset(newFileName, 0, 256 * sizeof(char));
            sprintf_s(newFileName, "%s%s%s%d.force", dir.c_str(), fileName.c_str(), meshName.c_str(), ithFrame);
            ofstream out(newFileName);

            DecoSoftObject* softBody = static_cast<DecoSoftObject*>(allObjs[ithObj]);
            int numNodes = softBody->GetNumNodes();
            for (int ithNode = 0; ithNode < numNodes; ++ithNode)
            {
                vector3 pos = softBody->GetIthVertex(ithNode);
                if (softBody->mForces->mPerturbationForce[ithNode].lengthSqr() > EPSILON_FLOAT)
                    out << pos << " " << softBody->mForces->mPerturbationForce[ithNode] << " " << "0" << endl;
                if (softBody->mForces->mNodeInContact[ithNode])
                {
                    vector3 contactForce = softBody->mForces->mNormalContactForces[ithNode] + softBody->mForces->mFrictionContactForces[ithNode];
                    if (contactForce.lengthSqr() > EPSILON_FLOAT)
                        out << pos << " " << contactForce << " " << "1" << endl;
                }
          
            }
        }
        LOG(INFO) << "Finish Dumping " << ithFrame << "th frame.";
    }
}


void MoviePlayer::SaveGeometry(const string& filename)
{
    vector<DecoSceneObject*> allObjs;
    mScene->GetAllSceneObjects(allObjs);
    int numObjs = static_cast<int>(allObjs.size());
    string dir = "../../../obj/";
    char newFileName[256];
    string fileName = CleanPath(filename);

    for (int ithFrame = 0; ithFrame < mNumFrames; ++ithFrame)
    {
		PlayIthFrame(ithFrame);
        for (int ithObj = 0; ithObj < numObjs; ++ithObj)
        {
            string meshName = CleanPath(allObjs[ithObj]->GetName());
            memset(newFileName, 0, 256 * sizeof(char));
            sprintf_s(newFileName, "%s%s%s%d.obj", dir.c_str(), fileName.c_str(), meshName.c_str(), ithFrame);
			
            if (strstr(meshName.c_str(), "ground") == NULL)
                allObjs[ithObj]->GetRenderData()->DumpToObjFile(newFileName);
            else if (ithFrame == 0)
                allObjs[ithObj]->GetRenderData()->DumpToObjFile(newFileName);
        }
        LOG(INFO) << "Finish Dumping " << ithFrame << "th frame.";
    }
}

void MoviePlayer::Play()
{
	if (mIsPaused)
		return;

	incrementFrameCounter();
	PlayIthFrame(mIthFrame);
}

void MoviePlayer::PlayIthFrame(int ithFrame)
{
	LOG(INFO) << "Playing " << ithFrame << "th frame.";
	vector<DecoSceneObject*> allObjs;
	mScene->GetAllSceneObjects(allObjs);
	int numObjs = static_cast<int>(allObjs.size());
	for (int i = 0; i < numObjs; ++i)
	{
		allObjs[i]->PlaybackMovie(ithFrame);
	}
}
void MoviePlayer::Pause()
{
	mIsPaused = !mIsPaused;
}

void MoviePlayer::incrementFrameCounter(bool isLoop)
{
	mIthFrame++;
	if (mIthFrame >= mNumFrames)
	{
		if (isLoop)
			mIthFrame %= mNumFrames;
		else
			mIthFrame--;
	}

}

void MoviePlayer::decrementFrameCounter(bool isLoop)
{
	mIthFrame--;
	if (mIthFrame < 0)
	{
		if (isLoop)
			mIthFrame = (mIthFrame + mNumFrames) % mNumFrames;
		else
			mIthFrame = 0;
	}

}

