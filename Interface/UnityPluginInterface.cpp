#if RECASTUNITY_EXPORTS // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

#include "UnityTileMesh.h"
#include "UnityInputMesh.h"
#include "InputGeom.h"
#include "DebugDraw.h"
#include "DetourCommon.h"

// ------------------------------------------------------------------------
// Plugin itself

struct Vec3 
{
	Vec3() : x(0), y(0), z(0) {}

	Vec3(const Vec3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {}

	float x, y, z;
};

// Link following functions C-style (required for plugins)
extern "C"
{
	UnityTileMesh*		GUnityTileMesh = 0;
	rcMeshLoaderObj*	GUnityInputMesh = 0;
	InputGeom*			GUnityInputGeom = 0;
	duDisplayList*		GUnityDebugDraw = 0;

	BuildContext GBuildCtx;

	int GVerticesCap = 0;
	int GTriangleCap = 0;
	int GIndicesCount = 0;

	void EXPORT_API CreateLog(int bufferMaxSize, int maxMessageCount)
	{
		
	}

	int EXPORT_API GetGwNavLogMessageCount()
	{
		return 0;
	}

	EXPORT_API const char* GetGwNavLog(int messageIndex, int* out_messageSize)
	{
		return 0;
	}

	void EXPORT_API FlushGwNavLog()
	{

	}

	void EXPORT_API DestroyLog()
	{

	}

	bool EXPORT_API NavGeneration_Init()
	{
		GUnityTileMesh = new UnityTileMesh();

		GUnityInputMesh = new rcMeshLoaderObj();

		GUnityInputGeom = new InputGeom();

		return true;
	}

	void EXPORT_API NavGeneration_DeInit()
	{
		if ( 0 != GUnityTileMesh )
		{
			delete GUnityTileMesh;
			GUnityTileMesh = 0;
		}

		if (GUnityInputMesh)
		{
			delete GUnityInputMesh;
			GUnityInputMesh = 0;
		}

		if (GUnityInputGeom)
		{
			delete GUnityInputGeom;
			GUnityInputGeom = 0;
		}

		if (GUnityDebugDraw)
		{
			delete GUnityDebugDraw;
			GUnityDebugDraw = 0;
		}
	}

	void EXPORT_API InitGenerator()
	{
		GUnityTileMesh->setContext(&GBuildCtx);
		
		if (GUnityInputGeom)
		{
			delete GUnityInputGeom;
			GUnityInputGeom = new InputGeom();
		}

		if (GUnityInputMesh)
		{
			delete GUnityInputMesh;
			GUnityInputMesh = new rcMeshLoaderObj();
		}

		GVerticesCap = 0;
		GTriangleCap = 0;
		GIndicesCount = 0;
	}

	void EXPORT_API PushTriangle(Vec3 A, Vec3 B, Vec3 C)
	{
		GUnityInputMesh->addVertex(A.x, A.y, A.z, GVerticesCap);
		GUnityInputMesh->addVertex(B.x, B.y, B.z, GVerticesCap);
		GUnityInputMesh->addVertex(C.x, C.y, C.z, GVerticesCap);

		GUnityInputMesh->addTriangle(GIndicesCount, GIndicesCount+1, GIndicesCount+2, GTriangleCap);

		GIndicesCount += 3;
	}

	void EXPORT_API PushTriangleWithNavTag(Vec3 A, Vec3 B, Vec3 C)
	{
		PushTriangle(A, B, C);
	}

	void EXPORT_API PushSeedPoint(Vec3 pos)
	{

	}

	bool EXPORT_API Generate(NavMeshBuildSettings params, char* generationName)
	{
		rcContext context;

		GUnityInputGeom->loadMesh(&context, GUnityInputMesh);

		GUnityTileMesh->handleMeshChanged(GUnityInputGeom)
			;
		GUnityTileMesh->handleSettings(params);

		GUnityTileMesh->handleBuild();

		return true;
	}

	int EXPORT_API GetGeneratedDataSizeInBytes()
	{
		return 0;
	}

	bool EXPORT_API GetGeneratedData(int** buffer, int* bufferSize)
	{
		GUnityTileMesh->saveAll(buffer, bufferSize);
		
		return true;
	}

	int EXPORT_API GetNavMeshTriangleCount()
	{

		return 0;
	}

	bool EXPORT_API LoadNavDataImmediate(int* memory)
	{
		GUnityTileMesh->cleanup();
		GUnityTileMesh->loadAll((const char*)memory);
		return true;
	}

	/// Call this at runtime to removeAllNavData.
	/// At generation time, call WorldRemoveAndCancelAll to allow adding NadData with different params than previously loaded NavData.
	bool EXPORT_API RemoveAllNavData()
	{
		return true;
	}

	bool EXPORT_API BuildDatabaseGeometry()
	{
		return true;
	}

	unsigned int EXPORT_API GetDatabaseTriangleCount()
	{
		if (!GUnityDebugDraw)
			GUnityDebugDraw = new duDisplayList();

		GUnityDebugDraw->clear();

		if (GUnityDebugDraw->m_size == 0)
		{
			if (!GUnityTileMesh->getNavMesh())
				return 0;

			GUnityTileMesh->handleRender(GUnityDebugDraw, false);
		}

		return GUnityDebugDraw->m_size/3;
	}

	class GwNavTriangle
	{
	public:
		GwNavTriangle() {}

		Vec3 A;
		Vec3 B;
		Vec3 C; 
	};

	class GwNavColor
	{
	public:
		GwNavColor() : r(0), g(0), b(0), a(0) {}

		void Set(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
		{
			r = red;
			g = green;
			b = blue;
			a = alpha;
		}

		unsigned char r;
		unsigned char g;
		unsigned char b;
		unsigned char a;
	};

	struct GwNavUnityVisualTriangle
	{
		GwNavTriangle m_triangle;
		GwNavColor m_color;
	};


 	bool EXPORT_API GetDatabaseTriangle(unsigned int triangleIndex, GwNavUnityVisualTriangle* out_triangle)
 	{
		if (!GUnityDebugDraw)
			return false;

		if (triangleIndex >= (GUnityDebugDraw->m_size/3))
			return false;

		float* A = &GUnityDebugDraw->m_pos[9*triangleIndex];
		float* B = &GUnityDebugDraw->m_pos[9*triangleIndex+3];
		float* C = &GUnityDebugDraw->m_pos[9*triangleIndex+6];

		unsigned int color = GUnityDebugDraw->m_color[3*triangleIndex];

		if (out_triangle)
		{
			out_triangle->m_triangle.A.x = A[0];
			out_triangle->m_triangle.A.y = A[1];
			out_triangle->m_triangle.A.z = A[2];

			out_triangle->m_triangle.B.x = B[0];
			out_triangle->m_triangle.B.y = B[1];
			out_triangle->m_triangle.B.z = B[2];

			out_triangle->m_triangle.C.x = C[0];
			out_triangle->m_triangle.C.y = C[1];
			out_triangle->m_triangle.C.z = C[2];

			out_triangle->m_color.Set(0,48,64,196);
		}
		return true;
 	}

	int EXPORT_API GetPolygonVertexCount()
	{
		if (!GUnityDebugDraw)
			GUnityDebugDraw = new duDisplayList();

		GUnityDebugDraw->clear();

		if (GUnityDebugDraw->m_size == 0)
		{
			if (!GUnityTileMesh->getNavMesh())
				return 0;

			GUnityTileMesh->handleRenderdPolyBoundaries(GUnityDebugDraw);
		}

		return GUnityDebugDraw->m_size;
	}

	bool EXPORT_API GetPolygonVertex(unsigned int vertexIndex, Vec3* vertex, GwNavColor* color)
	{
		if (!GUnityDebugDraw)
			return false;

		if (vertexIndex >= GUnityDebugDraw->m_size)
			return false;

		if (vertex)
		{
			vertex->x = GUnityDebugDraw->m_pos[3*vertexIndex];
			vertex->y = GUnityDebugDraw->m_pos[3*vertexIndex+1];
			vertex->z = GUnityDebugDraw->m_pos[3*vertexIndex+2];
		}

		if (color)
		{
			unsigned int col = GUnityDebugDraw->m_color[vertexIndex];
			if (col == duRGBA(0,48,64,32))
				color->Set(0, 48, 64, 32);
			else
				color->Set(0,48,64,220);
		}
		return true;
	}

	int EXPORT_API CalcSmoothPath(Vec3* start, Vec3* end)
	{
		GUnityTileMesh->pathFind((const float*)start, (const float*)end);
		dtStatus pathFindStatus = GUnityTileMesh->getPathFindStatus();
		if (pathFindStatus != DT_SUCCESS)
		{
			return 0;
		}

		return GUnityTileMesh->getSmoothPathNum();
	}

	void EXPORT_API GetSmoothPath(Vec3 dest[])
	{
		int nsmoothPath = GUnityTileMesh->getSmoothPathNum();
		if (nsmoothPath != 0)
		{
			float * smoothPath = GUnityTileMesh->getSmoothPath();
			for (int i = 0; i < nsmoothPath; i++)
			{
				float * destp = (float *)&dest[i];
				dtVcopy(destp, smoothPath + i * 3);
			}

			//memcpy(dest, smoothPath, sizeof(float *) * nsmoothPath * 3);
		}
	}

} // end of export C block