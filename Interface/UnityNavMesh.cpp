//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "UnityNavMesh.h"
#include "InputGeom.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}

static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
	const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;

	// Find furthest common polygon.
	for (int i = npath - 1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited - 1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;

	// Concatenate paths.	

	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath + 1, npath);
	int size = rcMax(0, npath - orig);
	if (req + size > maxPath)
		size = maxPath - req;
	if (size)
		memmove(path + req, path + orig, size * sizeof(dtPolyRef));

	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited - 1) - i];

	return req + size;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
	const float minTargetDist,
	const dtPolyRef* path, const int pathSize,
	float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
	float* outPoints = 0, int* outPointCount = 0)
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS * 3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
		steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;

	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i * 3], &steerPath[i * 3]);
	}


	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;

	dtVcopy(steerPos, &steerPath[ns * 3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];

	return true;
}

UnityNavMesh::UnityNavMesh() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	//m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	//m_crowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;

	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;

	m_neighbourhoodRadius = 2.5f;
}

UnityNavMesh::~UnityNavMesh()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	//dtFreeCrowd(m_crowd);

	SAFE_DELETE(m_tool);
	for (int i = 0; i < MAX_TOOLS; i++)
		SAFE_DELETE(m_toolStates[i]);
}

void UnityNavMesh::setTool(SampleTool* tool)
{
	SAFE_DELETE(m_tool);
	m_tool = tool;
	if (tool)
		m_tool->init(this);
}


void UnityNavMesh::recalc()
{
	if (!m_navMesh)
		return;

	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;

	if (m_eposSet)
		m_navQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	else
		m_endRef = 0;

	m_pathFindStatus = DT_FAILURE;

	m_pathIterNum = 0;
	if (m_sposSet && m_eposSet && m_startRef && m_endRef)
	{
#ifdef DUMP_REQS
		printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
			m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2],
			m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif

		m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);

		m_nsmoothPath = 0;

		if (m_npolys)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			dtPolyRef polys[MAX_POLYS];
			memcpy(polys, m_polys, sizeof(dtPolyRef)*m_npolys);
			int npolys = m_npolys;

			float iterPos[3], targetPos[3];
			m_navQuery->closestPointOnPolyBoundary(m_startRef, m_spos, iterPos);
			m_navQuery->closestPointOnPolyBoundary(polys[npolys - 1], m_epos, targetPos);

			static const float STEP_SIZE = 0.5f;
			static const float SLOP = 0.01f;

			m_nsmoothPath = 0;

			dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
			m_nsmoothPath++;

			// Move towards target a small advancement at a time until target reached or
			// when ran out of memory to store the path.
			while (npolys && m_nsmoothPath < MAX_SMOOTH)
			{
				// Find location to steer towards.
				float steerPos[3];
				unsigned char steerPosFlag;
				dtPolyRef steerPosRef;

				if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP,
					polys, npolys, steerPos, steerPosFlag, steerPosRef))
					break;

				bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
				bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

				// Find movement delta.
				float delta[3], len;
				dtVsub(delta, steerPos, iterPos);
				len = dtSqrt(dtVdot(delta, delta));
				// If the steer target is end of path or off-mesh link, do not move past the location.
				if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
					len = 1;
				else
					len = STEP_SIZE / len;
				float moveTgt[3];
				dtVmad(moveTgt, iterPos, delta, len);

				// Move
				float result[3];
				dtPolyRef visited[16];
				int nvisited = 0;
				m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
					result, visited, &nvisited, 16);

				npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
				float h = 0;
				m_navQuery->getPolyHeight(polys[0], result, &h);
				result[1] = h;
				dtVcopy(iterPos, result);

				// Handle end of path and off-mesh links when close enough.
				if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
				{
					// Reached end of path.
					dtVcopy(iterPos, targetPos);
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
						m_nsmoothPath++;
					}
					break;
				}
				else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
				{
					// Reached off-mesh connection.
					float startPos[3], endPos[3];

					// Advance the path up to and over the off-mesh connection.
					dtPolyRef prevRef = 0, polyRef = polys[0];
					int npos = 0;
					while (npos < npolys && polyRef != steerPosRef)
					{
						prevRef = polyRef;
						polyRef = polys[npos];
						npos++;
					}
					for (int i = npos; i < npolys; ++i)
						polys[i - npos] = polys[i];
					npolys -= npos;

					// Handle the connection.
					dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
					if (dtStatusSucceed(status))
					{
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos);
							m_nsmoothPath++;
							// Hack to make the dotted path not visible during off-mesh connection.
							if (m_nsmoothPath & 1)
							{
								dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos);
								m_nsmoothPath++;
							}
						}
						// Move position at the other side of the off-mesh link.
						dtVcopy(iterPos, endPos);
						float h;
						m_navQuery->getPolyHeight(polys[0], iterPos, &h);
						iterPos[1] = h;
					}
				}

				// Store results.
				if (m_nsmoothPath < MAX_SMOOTH)
				{
					dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos);
					m_nsmoothPath++;
				}
			}

			m_pathFindStatus = DT_SUCCESS;
		}

	}
	else
	{
		m_npolys = 0;
		m_nsmoothPath = 0;
	}
}

void UnityNavMesh::handleSettings()
{
}

void UnityNavMesh::handleTools()
{
}

void UnityNavMesh::handleDebugMode()
{
}

void UnityNavMesh::handleRender( duDebugDraw* dd, bool showDetail )
{
// 	if (!m_geom)
// 		return;
	//duDisplayList

	if (!m_navMesh)
		return;

	duDebugDrawNavMesh(dd, *m_navMesh, 0);
	
// 	DebugDrawGL dd;
// 		
// 	// Draw mesh
// 	duDebugDrawTriMesh(&dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
// 					   m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.0f);
// 	// Draw bounds
// 	const float* bmin = m_geom->getMeshBoundsMin();
// 	const float* bmax = m_geom->getMeshBoundsMax();
// 	duDebugDrawBoxWire(&dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void UnityNavMesh::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void UnityNavMesh::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* UnityNavMesh::getBoundsMin()
{
	if (!m_geom) return 0;
	return m_geom->getMeshBoundsMin();
}

const float* UnityNavMesh::getBoundsMax()
{
	if (!m_geom) return 0;
	return m_geom->getMeshBoundsMax();
}

void UnityNavMesh::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 8;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
	m_monotonePartitioning = false;
}

void UnityNavMesh::handleCommonSettings(const NavMeshBuildSettings& params)
{
	m_cellSize = 2.0f * params.agentRadius * 0.01f * params.widthInaccuracy;
	m_cellHeight = params.agentHeight * 0.01f * params.heightInaccuracy;

	m_monotonePartitioning = true;

	if (m_geom)
	{
		const float* bmin = m_geom->getMeshBoundsMin();
		const float* bmax = m_geom->getMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		//imguiValue(text);
	}
	
// 	imguiSeparator();
// 	imguiLabel("Agent");
// 	imguiSlider("Height", &m_agentHeight, 0.1f, 5.0f, 0.1f);
// 	imguiSlider("Radius", &m_agentRadius, 0.0f, 5.0f, 0.1f);
// 	imguiSlider("Max Climb", &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
// 	imguiSlider("Max Slope", &m_agentMaxSlope, 0.0f, 90.0f, 1.0f);
	m_agentHeight = params.agentHeight;
	m_agentRadius = params.agentRadius;
	m_agentMaxClimb = params.agentClimb;
	m_agentMaxSlope = rcMax(params.agentSlope, 0.1f);
// 	
// 	imguiSeparator();
// 	imguiLabel("Region");
// 	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.0f);
// 	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.0f);

	m_regionMinSize = params.minRegionArea;
	m_regionMergeSize = 400;

// 	imguiSeparator();
// 	imguiLabel("Partitioning");
// 	if (imguiCheck("Watershed", m_partitionType == SAMPLE_PARTITION_WATERSHED))
// 		m_partitionType = SAMPLE_PARTITION_WATERSHED;
// 	if (imguiCheck("Monotone", m_partitionType == SAMPLE_PARTITION_MONOTONE))
// 		m_partitionType = SAMPLE_PARTITION_MONOTONE;
// 	if (imguiCheck("Layers", m_partitionType == SAMPLE_PARTITION_LAYERS))
// 		m_partitionType = SAMPLE_PARTITION_LAYERS;
// 	
// 	imguiSeparator();
// 	imguiLabel("Polygonization");
// 	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.0f);
// 	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);
// 	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.0f);		

	m_edgeMaxLen = 0;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = DT_VERTS_PER_POLYGON;

// 	imguiSeparator();
// 	imguiLabel("Detail Mesh");
// 	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 16.0f, 1.0f);
// 	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 16.0f, 1.0f);
	
	m_detailSampleDist = m_cellSize * 6.0f;
	m_detailSampleMaxError = m_cellHeight * 1.0f;

// 	imguiSeparator();
}

void UnityNavMesh::pathFind(const float* s, const float* e)
{
	m_sposSet = true;
	dtVcopy(m_spos, s);

	m_eposSet = true;
	dtVcopy(m_epos, e);

	recalc();
}

void UnityNavMesh::handleClick(const float* s, const float* p, bool shift)
{
	if (shift)
	{
		m_sposSet = true;
		dtVcopy(m_spos, p);
	}
	else
	{
		m_eposSet = true;
		dtVcopy(m_epos, p);
	}
	recalc();

	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void UnityNavMesh::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void UnityNavMesh::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool UnityNavMesh::handleBuild()
{
	return true;
}

void UnityNavMesh::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void UnityNavMesh::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void UnityNavMesh::initToolStates(UnityNavMesh* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void UnityNavMesh::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void UnityNavMesh::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void UnityNavMesh::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

void UnityNavMesh::handleRenderdPolyBoundaries(duDebugDraw* dd)
{
	if (!m_navMesh)
		return;

	duDebugDrawNavMeshPolyBoundaries(dd, *m_navMesh);
}

#if defined(WIN32)

// Win32
#include <windows.h>

TimeVal getPerfTime()
{
	__int64 count;
	QueryPerformanceCounter((LARGE_INTEGER*)&count);
	return count;
}

int getPerfDeltaTimeUsec(const TimeVal start, const TimeVal end)
{
	static __int64 freq = 0;
	if (freq == 0)
		QueryPerformanceFrequency((LARGE_INTEGER*)&freq);
	__int64 elapsed = end - start;
	return (int)(elapsed*1000000 / freq);
}

#else

// Linux, BSD, OSX

#include <sys/time.h>

TimeVal getPerfTime()
{
	timeval now;
	gettimeofday(&now, 0);
	return (TimeVal)now.tv_sec*1000000L + (TimeVal)now.tv_usec;
}

int getPerfDeltaTimeUsec(const TimeVal start, const TimeVal end)
{
	return (int)(end - start);
}

#endif

#ifdef WIN32
#	define snprintf _snprintf
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext() :
m_messageCount(0),
	m_textPoolSize(0)
{
	resetTimers();
}

BuildContext::~BuildContext()
{
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
	m_messageCount = 0;
	m_textPoolSize = 0;
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;
	char* dst = &m_textPool[m_textPoolSize];
	int n = TEXT_POOL_SIZE - m_textPoolSize;
	if (n < 2)
		return;
	char* cat = dst;
	char* text = dst+1;
	const int maxtext = n-1;
	// Store category
	*cat = (char)category;
	// Store message
	const int count = rcMin(len+1, maxtext);
	memcpy(text, msg, count);
	text[count-1] = '\0';
	m_textPoolSize += 1 + count;
	m_messages[m_messageCount++] = dst;
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
		m_accTime[i] = -1;
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
	m_startTime[label] = getPerfTime();
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal endTime = getPerfTime();
	const int deltaTime = (int)(endTime - m_startTime[label]);
	if (m_accTime[label] == -1)
		m_accTime[label] = deltaTime;
	else
		m_accTime[label] += deltaTime;
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return m_accTime[label];
}

void BuildContext::dumpLog(const char* format, ...)
{
	// Print header.
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");

	// Print messages
	const int TAB_STOPS[4] = { 28, 36, 44, 52 };
	for (int i = 0; i < m_messageCount; ++i)
	{
		const char* msg = m_messages[i]+1;
		int n = 0;
		while (*msg)
		{
			if (*msg == '\t')
			{
				int count = 1;
				for (int j = 0; j < 4; ++j)
				{
					if (n < TAB_STOPS[j])
					{
						count = TAB_STOPS[j] - n;
						break;
					}
				}
				while (--count)
				{
					putchar(' ');
					n++;
				}
			}
			else
			{
				putchar(*msg);
				n++;
			}
			msg++;
		}
		putchar('\n');
	}
}

int BuildContext::getLogCount() const
{
	return m_messageCount;
}

const char* BuildContext::getLogText(const int i) const
{
	return m_messages[i]+1;
}