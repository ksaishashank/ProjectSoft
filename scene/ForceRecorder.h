#ifndef _FORCE_RECORDER_H
#define _FROCE_RECORDER_H

#include "Render/RenderInterface.h"
#include "ForceRecorderRenderType.h"

class DecoSoftObject;

class ForceRecorder
{
	friend class DecoSoftObject;
	friend class DecoSoftObjectCorotationLinearFEM;
	friend class ActiveSoftBody;
    friend class MoviePlayer;
public:
	ForceRecorder();
	~ForceRecorder();
	void SetRecordingObject(DecoSoftObject* obj);
	void SetRenderType(ForceRecorderRenderType type);
	void SetPlayerVersion(double version);
	void Render(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void ToggleRenderType(ForceRecorderRenderType type);
	void SetNumNodes(int numNodes);
	void CopyContent(const ForceRecorder& rhs);
	vector3 GetNetInternalForce() const;
	vector3 GetNetExternalForce() const;
	vector3 GetNetForce() const;
	vector3 GetNetInternalTorque() const;
	vector3 GetNetExternalTorque() const;
	vector3 GetNetContactForce() const;
	vector3 GetNetContactTorque() const;
	vector3 GetNetTorque() const;
    void Push();
    void Pop();
	void Check();
	void ClearAllForce();
	friend DecoArchive& operator<< (DecoArchive& Ar, const ForceRecorder& force);
	friend DecoArchive& operator>> (DecoArchive& Ar, ForceRecorder& force);

private:
	vector<vector3> mNormalContactForces; //for rendering and debug only
	vector<vector3> mFrictionContactForces; 
	vector<vector3> mTotalForces;
	vector<int> mNodeInContact;
	vector<vector3> mMuscleForce;
	vector<vector3> mGravityForce;
	vector<vector3> mElasticForce;
	vector<vector3> mContactPolygon;
	vector<vector3> mPerturbationForce;

    vector<vector3> mBackupNormalContactForces; //for rendering and debug only
    vector<vector3> mBackupFrictionContactForces; 
    vector<vector3> mBackupTotalForces;
    vector<int> mBackupNodeInContact;
    vector<vector3> mBackupMuscleForce;
    vector<vector3> mBackupGravityForce;
    vector<vector3> mBackupElasticForce;
    vector<vector3> mBackupContactPolygon;
    vector<vector3> mBackupPerturbationForce;

	ForceRecorderRenderType mType;
	DecoSoftObject* mObj;
	double mVersion;
	
	void drawPerturbationForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawContactForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawGravityForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawElasticForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawMuscleForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawTotalForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawExternalForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawInternalForce(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
    void drawContactForceOnCOP(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawTotalTorque(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawExternalTorque(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;
	void drawInternalTorque(DecoRenderInterface* RI, DecoLight** Lights = NULL, INT numLight = 0, DecoDrawType DrawType = DT_SolidPolygon) const;

};

#endif