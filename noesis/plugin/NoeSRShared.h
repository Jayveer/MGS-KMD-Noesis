#if !defined(_NOESRSHARED_H)

#define _NOESRSHARED_H

#include <vector>

//unlike the rest of the Noesis API, these structures are not padded for backward-maintainability, and architecture here is subject to change. expect the possibility that your usage of
//this API will be incompatible with future versions of Noesis.

class CNoeSRContext;

enum ENoeSRBufferType
{
	kNSRBT_RgbaFloat128 = 0,
	kNSRBT_Rgba32,
	kNSRBT_Virtual, //performs all raster ops and output processing, but doesn't allocate or write to a buffer
	kNSRBT_Count
};

enum ENoeSRDepthType
{
	kNSRDT_None = 0,
	kNSRDT_Float32,
	kNSRDT_Count
};

#define kSRClearFlag_Color (1 << 0)
#define kSRClearFlag_Depth (1 << 1)

struct SNoeSRBuffer
{
	int mWidth;
	int mHeight;
	ENoeSRBufferType mBufferType;
	void *mpBuffer;
	int mBufferSize;
	ENoeSRDepthType mDepthType;
	float *mpDepth;
	int mDepthSize;
};

struct SNoeSRVertClipNdc
{
	RichVec4 mModel;
	RichVec4 mClip;
	RichVec4 mNdc;
};
typedef std::vector<SNoeSRVertClipNdc> TNoeSRTransformedVerts;

struct SNoeSRTriangle
{
	static const unsigned int skTriFlag_None = 0;
	static const unsigned int skTriFlag_Clipped = (1 << 0);

	unsigned int mIndices[3];
	unsigned int mFlags;
	unsigned int mIndex; //original triangle index - depending on internal preprocessor configuration and clipping mode, many triangles may share a single mIndex
	union
	{
		//unclipped triangles will have computed inverse area, while clipped ones will have an index into the unclipped triangle list
		float mInvArea;
		unsigned int mUnclippedIndex;
	};

	RichVec2 mNdcMin;
	RichVec2 mNdcMax;
};
typedef std::vector<SNoeSRTriangle> TNoeSRTriangleList;

struct SNoeSRTriangleRenderData
{
	SNoeSRTriangleRenderData(noeRAPI_t *pRapi)
		: mpContext(NULL)
		, mpTri(NULL)
		, mpUnclippedTri(NULL)
		, mpVerts(NULL)
		, mPrimIndex(0)
		, mWindCcw(false)
		, mRenderingBackface(false)
		, mpUserData(NULL)
		, mpRapi(pRapi)
	{
	}

	CNoeSRContext *mpContext;

	const SNoeSRTriangle *mpTri;
	const SNoeSRTriangle *mpUnclippedTri;
	const TNoeSRTransformedVerts *mpVerts;

	int mPrimIndex;
	bool mWindCcw;
	bool mRenderingBackface;

	RichVec3 mBaryCoords;
	RichVec3 mFragPos;

	void *mpUserData;
	noeRAPI_t *mpRapi;
};
typedef bool (*TNoeSROutputColorCallback)(RichVec4 *pColorOut, const SNoeSRTriangleRenderData &trd);

enum ENoeSRCullMode
{
	kNoeSRCullMode_None = 0,
	kNoeSRCullMode_CW, //clockwise is considered front-facing
	kNoeSRCullMode_CCW //counter-clockwise is considered front-facing
};

enum ENoeSRClipMode
{
	kNoeSRClipMode_None = 0,
	kNoeSRClipMode_NearOnly,
	kNoeSRClipMode_AllPlanes
};

enum ENoeSRTriQMode
{
	kNoeSRTriQMode_CT = 0,
	kNoeSRTriQMode_CTC,
	kNoeSRTriQMode_UL
};

//query "NoesisMisc_GetNoeSRExternalAPI" with NPAPI_GetUserExtProc to get this interface:
//SNoeSRExternalAPI *NoesisMisc_GetNoeSRExternalAPI(void);
struct SNoeSRExternalAPI
{
	CNoeSRContext *(*CreateContext)(noeRAPI_t *pRapi);
	void (*DestroyContext)(CNoeSRContext *pContext);

	//must be called before any of the below API methods can be used. if no context is set when CreateContext, the created context will be set by default.
	void (*SetContext)(CNoeSRContext *pContext);
	CNoeSRContext *(*GetContext)();

	void (*SetSurfaceBuffer)(SNoeSRBuffer *pBuffer);
	SNoeSRBuffer *(*GetSurfaceBuffer)();

	const RichMat44 &(*GetMVP)();
	void (*SetMVP)(const RichMat44 &mvp);

	bool (*GetEnableProjection)();
	void (*SetEnableProjection)(const bool enableProjection);

	ENoeSRClipMode (*GetClipMode)();
	void (*SetClipMode)(const ENoeSRClipMode clipMode);

	//if disabled, will not confine rendering to the surface buffer. only allowed disabled with kNSRBT_Virtual.
	bool (*GetConfineToSurface)();
	void (*SetConfineToSurface)(const bool confineToSurface);

	//contrary to typical convention, the viewport transform is automatically scaled by the buffer dimensions.
	//if this is not desired, you'll need to compensate for it in the zw of the transform vector.
	const RichVec4 &(*GetViewportTransform)();
	void (*SetViewportTransform)(const RichVec4 &viewportTransform);

	bool (*GetViewportFlip)();
	void (*SetViewportFlip)(const bool viewportFlip);

	bool (*GetDepthTest)();
	void (*SetDepthTest)(const bool depthTest);

	bool (*GetDepthWrite)();
	void (*SetDepthWrite)(const bool depthWrite);

	ENoeSRCullMode (*GetCullMode)();
	void (*SetCullMode)(const ENoeSRCullMode cullMode);

	TNoeSROutputColorCallback (*GetOutputColorCallback)();
	void (*SetOutputColorCallback)(TNoeSROutputColorCallback pOutputColorCallback);

	void *(*GetUserData)();
	void (*SetUserData)(void *pUserData);

	SNoeSRBuffer *(*AllocateSurfaceBuffer)(const int width, const int height, const ENoeSRBufferType bufferType,
											const ENoeSRDepthType depthType);
	void (*DestroySurfaceBuffer)(SNoeSRBuffer *pBuffer);
	bool (*ResolveSurfaceBufferToTexture)(noesisTex_t *pTex, const SNoeSRBuffer *pSurfaceBuffer); //returns true on success

	void (*ClearBuffer)(const int clearFlags, const RichVec4 *pClearColor, const float clearDepth);

	void (*RenderTriangles)(const void *pTriangles, const int triangleCount, const int triangleIndexSize, const int triangleStride,
							const float *pPositions, const int positionCount, const int positionStride, const int positionElemCount);
	void (*RenderTriangleStrip)(const void *pIndices, const int indexCount, const int indexSize,
								const float *pPositions, const int positionCount, const int positionStride, const int positionElemCount);

	//by default, implementation will run single-threaded. invoke this to allow partitioning the target buffer into multiple buckets for asynchronous output.
	void (*InitializeThreadedBuckets)(const int bucketDivisor, const int threadCount);

	RichVec3 (*CalculatePerspectiveCorrectClippedBarycentricCoords)(const SNoeSRTriangleRenderData &trd);

	void (*TransformPoint)(RichVec4 &ndcOut, RichVec4 &dcOut, const RichVec4 &point);

	ENoeSRTriQMode (*GetTriQMode)();
	void (*SetTriQMode)(const ENoeSRTriQMode triQMode);
};

#endif //_NOESRSHARED_H
