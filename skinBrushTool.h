// ---------------------------------------------------------------------
//
//  skinBrushTool.h
//  skinBrushTool
//
//  Created by ingo on 11/18/18.
//  Copyright (c) 2018 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------
#include "functions.h"
#include "setOverloads.h"

#ifndef __skinBrushTool__skinBrushTool__
#define __skinBrushTool__skinBrushTool__

#include <iostream>
//#include <tbb/tbb.h>
// using namespace std;
#include <math.h>
#include <maya/M3dView.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MCursor.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>
#include <maya/MEulerRotation.h>
#include <maya/MEvent.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFnCamera.h>
#include <maya/MFnDoubleArrayData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnSingleIndexedComponent.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MFrameContext.h>
#include <maya/MGlobal.h>
#include <maya/MItDependencyGraph.h>
#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItSelectionList.h>
#include <maya/MMatrix.h>
#include <maya/MMeshIntersector.h>
#include <maya/MPointArray.h>
#include <maya/MPxContext.h>
#include <maya/MPxContextCommand.h>
#include <maya/MPxToolCommand.h>
#include <maya/MSelectionList.h>
#include <maya/MString.h>
#include <maya/MStringArray.h>
#include <maya/MSyntax.h>
#include <maya/MToolsInfo.h>
#include <maya/MUIDrawManager.h>

#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

// Macro for the press/drag/release methods in case there is nothing
// selected or the tool gets applied outside any geometry. If the actual
// MStatus would get returned an error can get listed in terminal on
// Linux. But it's unnecessary and needs to be avoided. Therefore a
// kSuccess is returned just for the sake of being invisible.
#define CHECK_MSTATUS_AND_RETURN_SILENT(status) \
    if (status != MStatus::kSuccess) return MStatus::kSuccess;

// ---------------------------------------------------------------------
// the tool
// ---------------------------------------------------------------------

class skinBrushTool : public MPxToolCommand {
   public:
    skinBrushTool();
    ~skinBrushTool();

    static void *creator();
    static MSyntax newSyntax();

    MStatus parseArgs(const MArgList &args);

    MStatus doIt(const MArgList &args);
    MStatus redoIt();
    MStatus undoIt();
    MStatus callBrushRefresh();
    MStatus finalize();

    bool isUndoable() const;

    // setting the attributes
    void setAffectSelected(bool value);
    void setColor(MColor color);
    void setCurve(int value);
    void setDepth(int value);
    void setDepthStart(int value);
    void setDrawBrush(bool value);
    void setDrawRange(bool value);
    void setEnterToolCommand(MString value);
    void setExitToolCommand(MString value);
    void setFractionOversampling(bool value);
    void setIgnoreLock(bool value);
    void setKeepShellsTogether(bool value);
    void setLineWidth(int value);
    void setMessage(int value);
    void setOversampling(int value);
    void setRange(double value);
    void setSize(double value);
    void setStrength(double value);
    void setSmoothStrength(double value);
    void setTolerance(double value);
    void setPruneWeights(double value);
    void setUndersampling(int value);
    void setVolume(bool value);
    void setStepLine(int value);
    void setCommandIndex(int value);
    void setSoloColor(int value);
    void setSoloColorType(int value);
    void setCoverage(bool value);
    void setPostSetting(bool value);

   public:
    void setInfluenceIndices(MIntArray indices);
    void setMesh(MDagPath dagPath);
    void setNormalize(bool value);
    void setSelection(MSelectionList selection, MSelectionList hilite);
    void setSkinCluster(MObject skinCluster);
    void setVertexComponents(MObject components);
    void setWeights(MDoubleArray weights);
    void setUnoVertices(MIntArray editVertsIndices);
    void setUnoLocks(MIntArray locks);
    void setRedoLocks(MIntArray locks);

   private:
    bool affectSelectedVal;
    MColor colorVal;
    int curveVal;
    int depthVal;
    int depthStartVal;
    bool drawBrushVal;
    bool drawRangeVal;
    MString enterToolCommandVal;
    MString exitToolCommandVal;
    bool fractionOversamplingVal;
    bool ignoreLockVal;
    bool keepShellsTogetherVal;
    int lineWidthVal;
    int messageVal;
    int oversamplingVal;
    double rangeVal;
    double sizeVal;
    double strengthVal, smoothStrengthVal;
    double toleranceVal;
    double pruneWeights;
    int undersamplingVal;
    bool volumeVal;

    int stepsToDrawLineVal;
    bool coverageVal;
    int influenceIndex = 0, commandIndex = 0, smoothRepeat = 3, smoothDepth = 1;
    int soloColorTypeVal = 1, soloColorVal = 0;  // 1 lava
    bool postSetting = true;

    MIntArray influenceIndices;
    MDagPath meshDag;
    bool normalize;
    MSelectionList redoHilite;
    MSelectionList redoSelection;
    MDoubleArray redoWeights;
    MObject skinObj;
    MSelectionList undoHilite;
    MSelectionList undoSelection;
    MDoubleArray undoWeights;
    MIntArray undoVertices;
    MObject vertexComponents;

    MIntArray undoLocks, redoLocks;
};

// ---------------------------------------------------------------------
// the context
// ---------------------------------------------------------------------

class SkinBrushContext : public MPxContext {
   public:
    SkinBrushContext();
    void toolOnSetup(MEvent &event);
    void toolOffCleanup();

    void getClassName(MString &name) const;

    MStatus doPress(MEvent &event);
    MStatus doDrag(MEvent &event);
    MStatus doRelease(MEvent &event);

    void drawCircle(MPoint point, MMatrix mat, double radius);

    // VP2.0
    MStatus doPress(MEvent &event, MHWRender::MUIDrawManager &drawManager,
                    const MHWRender::MFrameContext &context);
    MStatus doDrag(MEvent &event, MHWRender::MUIDrawManager &drawManager,
                   const MHWRender::MFrameContext &context);
    MStatus doRelease(MEvent &event, MHWRender::MUIDrawManager &drawManager,
                      const MHWRender::MFrameContext &context);
    // MStatus drawFeedback(MHWRender::MUIDrawManager& drawMgr,const MHWRender::MFrameContext&
    // context);
    MStatus drawTheMesh(MHWRender::MUIDrawManager &drawManager, MVector worldVector);

    MStatus doPtrMoved(MEvent &event, MHWRender::MUIDrawManager &drawManager,
                       const MHWRender::MFrameContext &context);
    int getHighestInfluence(int faceHit, MFloatPoint hitPoint);
    // bool tmpTestPrintClosestInfluence = false;
    int getClosestInfluenceToCursor(int screenX, int screenY);
    // common methods
    MStatus doPressCommon(MEvent event);
    MStatus doDragCommon(MEvent event);
    void doReleaseCommon(MEvent event);

    MStatus getMesh();
    void getConnectedVertices();
    MStatus getSelection(MDagPath &dagPath);
    MIntArray getSelectionVertices();
    MStatus getSkinCluster(MDagPath meshDag, MObject &skinClusterObj);
    // MStatus getAllWeights();
    void refreshJointsLocks();
    void refresh();
    void refreshTheseVertices(MIntArray verticesIndices);

    // MStatus getMirrorInfos();
    MStatus applyCommand(int influence, std::unordered_map<int, double> &valuesToSet,
                         bool storeUndo = true);
    MStatus applyCommandMirror(std::unordered_map<int, double> &valuesToSet);
    MStatus refreshColors(MIntArray &editVertsIndices, MColorArray &multiEditColors,
                          MColorArray &soloEditColors);
    MStatus editSoloColorSet();
    MColor getASoloColor(double val);
    MStatus refreshPointsNormals();

    void setColor(int vertexIndex, float value, MIntArray &editVertsIndices,
                  MColorArray &multiEditColors, MColorArray &soloEditColors);

    MStatus querySkinClusterValues(MObject skinCluster, MIntArray &verticesIndices, bool doColors);
    MStatus fillArrayValues(MObject skinCluster, bool doColors);

    void getSkinClusterAttributes(MObject skinCluster, unsigned int &maxInfluences,
                                  bool &maintainMaxInfluences, unsigned int &normalize);
    MIntArray getInfluenceIndices(MObject skinCluster, MDagPathArray &dagPaths);
    // bool getClosestIndex(MEvent event);

    bool computeHit(short screenPixelX, short screenPixelY, bool getNormal, int &faceHit,
                    MFloatPoint &hitPoint);
    bool expandHit(int faceHit, MFloatPoint hitPoint, std::unordered_map<int, float> &dicVertsDist);

    // MVector SkinBrushContext::getNormal(int vertexInd, int faceIndex);
    void growArrayOfHits(std::unordered_map<int, float> &dicVertsDist);
    void growArrayOfHitsFromCenters(std::unordered_map<int, float> &dicVertsDist,
                                    MFloatPointArray &AllHitPoints);

    // smooth computation
    MStatus performPaint(std::unordered_map<int, float> &dicVertsDist,
                         std::unordered_map<int, float> &dicVertsDistRed);
    void prepareArray(std::unordered_map<int, float> &dicVertsDist);

    // selection
    MStatus performSelect(MEvent event, MIntArray indices, MFloatArray distances);
    // flood
    void performFlood();

    MObject allVertexComponents(MDagPath meshDag);
    MIntArray sortIndicesByValues(MIntArray ids, MDoubleArray array);
    void getVerticesInRangeFast(int index, int hitIndex, MIntArray &indices, MFloatArray &values);

    void appendConnectedIndices(int index, MIntArray &indices);
    MIntArray getVerticesInVolume();
    void getVerticesInVolumeRange(int index, MIntArray volumeIndices, MIntArray &rangeIndices,
                                  MFloatArray &values);

    double getFalloffValue(double value, double strength);
    bool eventIsValid(MEvent event);

    bool onBoundary(int index);
    bool oppositeBoundaryIndex(MPoint point, MIntArray faces, MIntArray edges, int &index);
    bool getClosestFace(MPoint point, MIntArray faces, int &index);
    double averageEdgeLength(MIntArray edges);

    void setInViewMessage(bool display);

    // setting the attributes
    void setAffectSelected(bool value);
    void setColorR(float value);
    void setColorG(float value);
    void setColorB(float value);
    void setCurve(int value);
    void setDepth(int value);
    void setDepthStart(int value);
    void setDrawBrush(bool value);
    void setDrawRange(bool value);
    void setEnterToolCommand(MString value);
    void setExitToolCommand(MString value);
    void setFlood(double value);
    void setPickMaxInfluence(bool value);
    void setPickInfluence(bool value);
    void setFractionOversampling(bool value);
    void setIgnoreLock(bool value);
    void setKeepShellsTogether(bool value);
    void setLineWidth(int value);
    void setMessage(int value);
    void setOversampling(int value);
    void setRange(double value);
    void setSize(double value);
    void setStrength(double value);
    void setSmoothStrength(double value);
    void setTolerance(double value);
    void setUndersampling(int value);
    void setVolume(bool value);
    void setStepLine(int value);
    void setCoverage(bool value);
    void setInfluenceIndex(int value, bool selectInUI);
    void setCommandIndex(int value);
    void setSoloColor(int value);
    void setSoloColorType(int value);
    void setInfluenceByName(MString value);
    void setPostSetting(bool value);

    void setPruneWeights(double value);

    // getting the attributes
    bool getAffectSelected();
    float getColorR();
    float getColorG();
    float getColorB();
    int getCurve();
    int getDepth();
    int getDepthStart();
    bool getDrawBrush();
    bool getDrawRange();
    MString getEnterToolCommand();
    MString getExitToolCommand();
    bool getFractionOversampling();
    bool getIgnoreLock();
    bool getKeepShellsTogether();
    int getLineWidth();
    int getMessage();
    int getOversampling();
    double getRange();
    double getSize();
    double getStrength();
    double getSmoothStrength();
    double getPruneWeights();
    double getTolerance();
    int getUndersampling();
    bool getVolume();
    int getStepLine();
    bool getCoverage();
    int getInfluenceIndex();
    MString getInfluenceName();
    MString getSkinClusterName();
    int getCommandIndex();
    int getSoloColor();
    int getSoloColorType();
    bool getPostSetting();

   private:
    bool verbose = false;
    skinBrushTool *cmd;

    bool performBrush;
    int undersamplingSteps;

    // the tool settings
    bool affectSelectedVal;
    MColor colorVal = MColor(1.0, 0, 0);
    int curveVal;
    int depthVal;
    int depthStartVal;
    bool drawBrushVal;
    bool drawRangeVal;
    MString enterToolCommandVal;
    MString exitToolCommandVal;
    bool fractionOversamplingVal;
    bool ignoreLockVal;
    bool keepShellsTogetherVal;
    int lineWidthVal;
    int messageVal;
    int oversamplingVal;
    double rangeVal;
    double sizeVal;
    double strengthVal, smoothStrengthVal;
    double toleranceVal;
    int undersamplingVal;
    bool volumeVal;
    int stepsToDrawLineVal;
    bool coverageVal;
    // if we're asking to pick max influence
    bool pickMaxInfluenceVal = false, pickInfluenceVal = false;
    // for me yep ----
    int influenceIndex = 0, commandIndex = 0, smoothRepeat = 4, smoothDepth = 3;
    int soloColorTypeVal = 1, soloColorVal = 0;  // 1 lava
    bool postSetting = true;                     // we apply paint as ssons as attr is changed
    bool doNormalize = true;

    // brush settings for adjusting
    bool initAdjust;                 // True after the first drag event.
                                     // Controls the adjust direction for
                                     // the size and the strength.
    MFloatPoint surfacePointAdjust;  // Initital surface point of the press
                                     // event.
    MVector worldVectorAdjust;       // Initial view vector of the press
                                     // event.
    bool sizeAdjust;                 // True, if the size is set.
    double adjustValue;              // The new value for the size or
                                     // strength.

    M3dView view;
    unsigned int width;
    unsigned int height;
    short viewCenterX;
    short viewCenterY;

    // the cursor position
    short screenX;
    short screenY;
    short startScreenX;
    short startScreenY;

    MPointArray surfacePoints;  // The cursor positions on the mesh in
                                // world space.
    MVector worldVector;        // The view vector from the camera to
                                // the surface point.
    MVector normalVector;       // The normal vector to camera

    MFloatPoint centerOfBrush;  // store the center of the bursh to display

    float pressDistance;        // The closest distance to the mesh on
                                // the press event.
    float previousHitDistance;  // The closest distance to the mesh on

    MStatus selectionStatus;

    MFnMesh meshFn;
    MDagPath meshDag;
    unsigned int numVertices = 0, numFaces = 0;
    MIntArray vtxSelection;  // The currently selected vertices. This
                             // is used for flooding.

    MObject attrValue;
    MDoubleArray valuesForAttribute, paintArrayValues;  // the array of values to paint

    MMeshIntersector intersector;

    std::vector<bool> selectedIndices;  // The current vertex selection
                                        // in a non-sparse array
                                        // spanning all vertices.
                                        // Selected indices are set to
                                        // true.

    MObject allVtxCompObj;
    MObject smoothedCompObj;  // The single index component object for
                              // holding only the indices of the
                              // current stroke for undo/redo.

    // the skin cluster
    MObject skinObj;
    unsigned int influenceCount;
    MIntArray influenceIndices;
    MDagPathArray inflDagPaths;
    MStringArray inflNames;
    MIntArray inflNamePixelSize;
    bool maintainMaxInfluences;
    unsigned int maxInfluences;
    bool normalize;

    MSelectionList prevSelection;
    MSelectionList prevHilite;

    // guillaume values ----------
    MMeshIsectAccelParams accelParams;
    // MIntArray closestIndices;
    // MFloatArray closestDistances;
    bool foundBlurSkinAttribute = false;

    // skinCluster values --------------------------
    double pruneWeight;
    int nbJoints = 0, nbJointsBig = 0;
    MIntArray deformersIndices;
    MIntArray cpIds;  // the ids of the vertices passed as to update skin for
    std::vector<std::vector<std::pair<int, float>>> skin_weights_;
    MDoubleArray skinWeightList, fullUndoSkinWeightList;

    std::vector<bool> influenceLocks;
    MIntArray lockJoints, ignoreLockJoints, lockVertices;

    // mirror things -----
    bool mirrorIsActive = false;
    MIntArray mirrorInfluences, mirrorVertices;
    bool changeOfMirrorData = false;

    // colorSet ------------------------
    MDGModifier colorSetMod;
    MDoubleArray soloColorsValues;
    MColor lockVertColor = MColor((float)0.2, (float)0.2, (float)0.2);
    MString fullColorSet = MString("multiColorsSet");
    MString soloColorSet = MString("soloColorsSet");
    MString noColorSet = MString("noColorsSet");
    std::vector<float> intensityValues;  // (length, 0);

    float minSoloColor = 0.0;
    float maxSoloColor = 1.0;

    MColorArray multiCurrentColors, jointsColors,
        soloCurrentColors;  // lock vertices color are not stored inside these arrays

    std::vector<MIntArray> connectedVertices;  // use by MItMeshVertex getConnectedVertices
    std::vector<std::vector<int>> connectedSetVertices;

    std::vector<MIntArray> connectedFaces;  // use by MItMeshVertex getConnectedFaces
    std::vector<MIntArray> FaceToVertices;  // use by MItMeshVertex getConnectedVertices
    std::vector<std::vector<int>> FaceToVerticesSet;
    std::vector<std::vector<int>> normalsIds;  // vector of faces Ids normals
    std::vector<std::pair<int, int>> edgeVerticesIndices;
    MVectorArray verticesNormals;
    MIntArray verticesNormalsIndices;

    const float *rawNormals;
    const float *mayaRawPoints;
    MPointArray meshPoints;
    MIntArray triangleCounts, triangleVertices;
    // MFloatPointArray vertexArray; // the position of all the vertices
    int fullVertexListLength = 0;

    // HITs vairables ------------------------
    bool successFullHit = false;
    bool successFullDragHit = false;
    bool refreshDone = false;
    // int stepsLine = 10; // steps to draw the line
    std::unordered_map<int, float> dicVertsDistSTART, previousPaint;
    std::unordered_map<int, double> skinValuesToSet;
    std::set<int> verticesPainted;  // the vertices that have been painted for a redraw purpose

    int modifierNoneShiftControl = 0;  // store the modifier type

    // std::vector <std::pair<int, float>> faceVertsDistSTART;
    int previousfaceHit;  // the faceIndex that was hit during the press common

    bool doAddingDots = false;

    int biggestInfluence;  // for while we search for biggest influence
};

// ---------------------------------------------------------------------
// command to create the context
// ---------------------------------------------------------------------

class SkinBrushContextCmd : public MPxContextCommand {
   public:
    SkinBrushContextCmd();
    MPxContext *makeObj();
    static void *creator();
    MStatus appendSyntax();
    MStatus doEditFlags();
    MStatus doQueryFlags();

   protected:
    SkinBrushContext *smoothContext;
};

#endif

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2018 Ingo Clemens, brave rabbit
// brSkinBrush is under the terms of the MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Ingo Clemens    www.braverabbit.com
// ---------------------------------------------------------------------