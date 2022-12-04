#include "functions.h"
#include "enums.h"
#include <math.h>
#include <limits>


// Find the squared distance between two points in 3-space
coord_t distance_sq(const point_t& a, const point_t& b) {
    coord_t x = std::get<0>(a) - std::get<0>(b);
    coord_t y = std::get<1>(a) - std::get<1>(b);
    coord_t z = std::get<2>(a) - std::get<2>(b);
    return x * x + y * y + z * z;
}

// Find the distance between two points in 3-space
coord_t distance(const point_t& a, const point_t& b) {
    coord_t x = std::get<0>(a) - std::get<0>(b);
    coord_t y = std::get<1>(a) - std::get<1>(b);
    coord_t z = std::get<2>(a) - std::get<2>(b);
    return std::sqrt(x * x + y * y + z * z);
}


// Get all CVs from a nurbs surface, and transfer those positions to the vertices of a mesh
MStatus transferPointNurbsToMesh(MFnMesh& msh, MFnNurbsSurface& nurbsFn) {
    MStatus stat = MS::kSuccess;
    MPlug mshPnts = msh.findPlug("pnts", false, &stat);
    MPointArray allpts;

    bool VIsPeriodic = nurbsFn.formInV() == MFnNurbsSurface::kPeriodic;
    bool UIsPeriodic = nurbsFn.formInU() == MFnNurbsSurface::kPeriodic;
    if (VIsPeriodic || UIsPeriodic) {
        int numCVsInV = nurbsFn.numCVsInV();
        int numCVsInU = nurbsFn.numCVsInU();
        if (VIsPeriodic) numCVsInV -= nurbsFn.degreeV();
        if (UIsPeriodic) numCVsInU -= nurbsFn.degreeU();
        for (int uIndex = 0; uIndex < numCVsInU; uIndex++) {
            for (int vIndex = 0; vIndex < numCVsInV; vIndex++) {
                MPoint pt;
                nurbsFn.getCV(uIndex, vIndex, pt);
                allpts.append(pt);
            }
        }
    } else {
        stat = nurbsFn.getCVs(allpts);
    }
    msh.setPoints(allpts);
    return stat;
}


// Get the node connected to the origSurfNurbs plug
MStatus findNurbsTesselateOrig(MDagPath surfPath, MObject& origSurfObj, bool verbose) {
    if (verbose)
        MGlobal::displayInfo(MString(" |||| findNurbsTesselateOrig ||||"));
    MStatus stat;

    MFnDependencyNode deformedNameSurf(surfPath.node());
    MPlug outSurfPlug = deformedNameSurf.findPlug("origSurfNurbs", false, &stat);
    if (stat != MS::kSuccess) {
        return MS::kFailure;
    }

    MPlugArray connections;
    outSurfPlug.connectedTo(connections, false, true);

    MPlug conn = connections[0];
    if (verbose)
        MGlobal::displayInfo(MString("---- connected to is : ") + conn.name());

    MFnDependencyNode sourceNode;
    sourceNode.setObject(conn.node());
    if (verbose)
        MGlobal::displayInfo(MString("---- connected to is Name : ") + sourceNode.name());

    origSurfObj = sourceNode.object();
    return MS::kSuccess;
}


// Find the node connected to the nurbsTessellate plug
MStatus findNurbsTesselate(MDagPath NurbsPath, MObject& MeshObj, bool verbose) {
    if (verbose)
        MGlobal::displayInfo(MString(" ---- findNurbsTesselate ----"));
    MStatus stat;

    MFnDependencyNode deformedNameMesh(NurbsPath.node());
    MPlug outMeshPlug = deformedNameMesh.findPlug("nurbsTessellate", false, &stat);
    if (stat != MS::kSuccess) {
        return MS::kFailure;
    }

    MPlugArray connections;
    outMeshPlug.connectedTo(connections, false, true);
    MPlug conn = connections[0];
    if (verbose)
        MGlobal::displayInfo(MString("---- connected to is : ") + conn.name());

    MFnDependencyNode sourceNode;
    sourceNode.setObject(conn.node());
    if (verbose)
        MGlobal::displayInfo(MString("---- connected to is Name : ") + sourceNode.name());
    MeshObj = sourceNode.object();
    return MS::kSuccess;
}

// Given a skincluster MObject, find the MDagPath of the first mesh it's deforming
MStatus findMesh(MObject& skinCluster, MDagPath& theMeshPath, bool verbose) {
    if (verbose)
        MGlobal::displayInfo(MString(" ---- findMesh ----"));

    MFnSkinCluster theSkinCluster(skinCluster);
    MObjectArray objectsDeformed;
    theSkinCluster.getOutputGeometry(objectsDeformed);
    int objectsDeformedCount = objectsDeformed.length();
    bool doContinue = false;

    if (objectsDeformedCount == 0) {
        return MS::kFailure;
    }

    MDagPath::getAPathTo(objectsDeformed[0], theMeshPath);
    if (verbose) {
        MFnDependencyNode deformedNameMesh(objectsDeformed[0]);
        MString deformedNameMeshSTR = deformedNameMesh.name();
        if (verbose)
            MGlobal::displayInfo("     -> DEFORMING : " + deformedNameMeshSTR + "\n");
    }
    return MS::kSuccess;
}

// Given a skincluster MObject, get the orig mesh
MStatus findOrigMesh(MObject& skinCluster, MObject& origMesh, bool verbose) {
    if (verbose)
        MGlobal::displayInfo(MString(" ---- find Orig Mesh ----"));
    MFnSkinCluster theSkinCluster(skinCluster);
    MObjectArray objectsDeformed;
    theSkinCluster.getInputGeometry(objectsDeformed);
    origMesh = objectsDeformed[0];
    if (verbose) {
        MFnDependencyNode deformedNameMesh(origMesh);
        MGlobal::displayInfo("     -> DEFORMING : " + deformedNameMesh.name() + "\n");
    }
    return MS::kSuccess;
}

MStatus getListColorsJoints(MObject& skinCluster, int nbJoints,
                            MIntArray indicesForInfluenceObjects, MColorArray& jointsColors,
                            bool verbose) {
    MStatus stat = MS::kSuccess;

    // start
    jointsColors.clear();
    jointsColors.setLength(nbJoints);
    float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    for (int i = 0; i < nbJoints; ++i) {
        jointsColors.set(black, i);
    }

    //----------------------------------------------------------------
    MFnDependencyNode skinClusterDep(skinCluster);
    MPlug influenceColor_plug = skinClusterDep.findPlug("influenceColor", false, &stat);
    if (stat != MS::kSuccess) {
        MGlobal::displayError(MString("fail finding influenceColor plug "));
        return stat;
    }
    int nbElements = influenceColor_plug.numElements();

    for (int i = 0; i < nbElements; ++i) {  // for each joint

        MPlug colorPlug = influenceColor_plug.elementByPhysicalIndex(i);
        int logicalInd = colorPlug.logicalIndex();
        logicalInd = indicesForInfluenceObjects[logicalInd];
        if (logicalInd < 0 || logicalInd >= nbJoints) {
            MGlobal::displayError(MString("CRASH i : ") + i + MString("logical Index: ") +
                                  colorPlug.logicalIndex() +
                                  MString(" | indicesForInfluenceObjects ") + logicalInd);
            continue;
        }

        if (colorPlug.isConnected()) {
            MPlugArray connections;
            colorPlug.connectedTo(connections, true, false);
            if (connections.length() > 0) {
                MPlug theConn = connections[0];
                float element[4] = {theConn.child(0).asFloat(), theConn.child(1).asFloat(),
                                    theConn.child(2).asFloat(), 1};
                jointsColors.set(element, logicalInd);
            } else {
                jointsColors.set(black, logicalInd);
            }
        } else {
            // MGlobal::displayInfo(colorPlug.name());
            float element[4] = {colorPlug.child(0).asFloat(), colorPlug.child(1).asFloat(),
                                colorPlug.child(2).asFloat(), 1};
            jointsColors.set(element, logicalInd);
        }
    }
    return stat;
}

MStatus getListLockJoints(MObject& skinCluster, int nbJoints, MIntArray indicesForInfluenceObjects,
                          MIntArray& jointsLocks) {
    MStatus stat;

    MFnDependencyNode skinClusterDep(skinCluster);
    MPlug influenceLock_plug = skinClusterDep.findPlug("lockWeights", false);

    int nbPlugs = influenceLock_plug.numElements();
    jointsLocks.clear();
    jointsLocks.setLength(nbJoints);
    for (int i = 0; i < nbJoints; ++i) jointsLocks.set(0, i);

    for (int i = 0; i < nbPlugs; ++i) {
        MPlug lockPlug = influenceLock_plug.elementByPhysicalIndex(i);
        int isLocked = 0;
        if (lockPlug.isConnected()) {
            MPlugArray connections;
            lockPlug.connectedTo(connections, true, false);
            if (connections.length() > 0) {
                MPlug theConn = connections[0];
                isLocked = theConn.asInt();
            }
        } else {
            isLocked = lockPlug.asInt();
        }
        int logicalInd = lockPlug.logicalIndex();
        logicalInd = indicesForInfluenceObjects[logicalInd];
        if (logicalInd < 0 || logicalInd >= nbJoints) {
            MGlobal::displayError(MString("CRASH i : ") + i + MString("logical Index: ") +
                                  lockPlug.logicalIndex() +
                                  MString(" | indicesForInfluenceObjects ") + logicalInd);
            continue;
        }
        jointsLocks.set(isLocked, logicalInd);
        // MGlobal::displayInfo(lockPlug.name() + " " + isLocked);
    }
    return stat;
}

MStatus getListLockVertices(MObject& skinCluster, MIntArray& vertsLocks, MIntArray& lockedIndices) {
    MStatus stat;

    MFnSkinCluster theSkinCluster(skinCluster);
    MObjectArray objectsDeformed;
    theSkinCluster.getOutputGeometry(objectsDeformed);
    MFnDependencyNode deformedNameMesh(objectsDeformed[0]);
    MPlug lockedVerticesPlug = deformedNameMesh.findPlug("lockedVertices", false, &stat);
    if (MS::kSuccess != stat) {
        MGlobal::displayInfo(MString("cant find lockerdVertices plug"));
        return stat;
    }

    MFnDependencyNode skinClusterDep(skinCluster);
    MPlug weight_list_plug = skinClusterDep.findPlug("weightList", false);

    int nbVertices = weight_list_plug.numElements();

    MObject Data;
    stat = lockedVerticesPlug.getValue(Data);  // to get the attribute

    MFnIntArrayData intData(Data);
    MIntArray vertsLocksIndices = intData.array(&stat);
    vertsLocks.clear();
    vertsLocks = MIntArray(nbVertices, 0);
    for (unsigned int i = 0; i < vertsLocksIndices.length(); ++i) {
        vertsLocks[vertsLocksIndices[i]] = 1;
        lockedIndices.append(vertsLocksIndices[i]);
    }
    // MGlobal::displayInfo(MString(" getListLockVertices | ") + currentColorSet.name () + MString("
    // ") + vertsLocks.length());
    return stat;
}

MStatus editLocks(MObject& skinCluster, MIntArray& inputVertsToLock, bool addToLock,
                  MIntArray& vertsLocks) {
    MStatus stat;

    MFnSkinCluster theSkinCluster(skinCluster);
    MObjectArray objectsDeformed;
    theSkinCluster.getOutputGeometry(objectsDeformed);
    MFnDependencyNode deformedNameMesh(objectsDeformed[0]);
    MPlug lockedVerticesPlug = deformedNameMesh.findPlug("lockedVertices", false, &stat);
    if (MS::kSuccess != stat) {
        MGlobal::displayError(MString("cant find lockerdVertices plug"));
        return stat;
    }

    // now expand the array -----------------------
    int val = 0;
    if (addToLock) val = 1;
    for (unsigned int i = 0; i < inputVertsToLock.length(); ++i) {
        int vtx = inputVertsToLock[i];
        vertsLocks[vtx] = val;
    }
    MIntArray theArrayValues;
    for (unsigned int vtx = 0; vtx < vertsLocks.length(); ++vtx) {
        if (vertsLocks[vtx] == 1) theArrayValues.append(vtx);
    }
    // now set the value ---------------------------
    MFnIntArrayData tmpIntArray;
    auto tmpAttrSetter = tmpIntArray.create(theArrayValues);
    stat = lockedVerticesPlug.setValue(tmpAttrSetter);  // to set the attribute
    return stat;
}

MStatus editArray(ModifierCommands command, int influence, int nbJoints, MIntArray& lockJoints,
                  MDoubleArray& fullWeightArray, std::map<int, double>& valuesToSet,
                  MDoubleArray& theWeights, bool normalize, double mutliplier, bool verbose) {
    MStatus stat;
    // 0 Add - 1 Remove - 2 AddPercent - 3 Absolute - 4 Smooth - 5 Sharpen - 6 LockVertices - 7
    // UnLockVertices
    //
    if (verbose)
        MGlobal::displayInfo(MString("-> editArray | command ") + static_cast<int>(command) +
                             MString(" | influence ") + influence);
    if (verbose)
        MGlobal::displayInfo(MString("-> editArray | nbJoints ") + nbJoints +
                             MString(" | lockJoints ") + lockJoints.length());
    if (lockJoints.length() < nbJoints) {
        MGlobal::displayInfo(MString("-> editArray FAILED | nbJoints ") + nbJoints +
                             MString(" | lockJoints ") + lockJoints.length());
        return MStatus::kFailure;
    }
    if (verbose)
        MGlobal::displayInfo(MString("-> editArray | theWeights ") + theWeights.length() +
                             MString(" | fullWeightArray ") + fullWeightArray.length());
    if (command == ModifierCommands::Sharpen) {
        int i = 0;
        for (const auto& elem : valuesToSet) {
            int theVert = elem.first;
            double theVal = mutliplier * elem.second + 1.0;
            double substract = theVal / nbJoints;
            MDoubleArray producedWeigths(nbJoints, 0.0);
            double totalBaseVtxUnlock = 0.0, totalBaseVtxLock = 0.0;
            ;
            double totalVtxUnlock = 0.0, totalVtxLock = 0.0;
            for (int j = 0; j < nbJoints; ++j) {
                // check the zero val ----------
                double currentW = fullWeightArray[theVert * nbJoints + j];
                double targetW = (currentW * theVal) - substract;
                targetW = std::max(0.0, std::min(targetW, 1.0));  // clamp
                producedWeigths.set(targetW, j);

                if (lockJoints[j] == 0) {  // unlock
                    totalBaseVtxUnlock += currentW;
                    totalVtxUnlock += targetW;
                } else {
                    totalBaseVtxLock += currentW;
                    totalVtxLock += targetW;
                }
            }
            // now normalize for lockJoints
            double normalizedValueAvailable = 1.0 - totalBaseVtxLock;
            if (normalizedValueAvailable > 0.0 &&
                totalVtxUnlock > 0.0) {  // we have room to set weights
                double mult = normalizedValueAvailable / totalVtxUnlock;
                for (unsigned int j = 0; j < nbJoints; ++j) {
                    double currentW = fullWeightArray[theVert * nbJoints + j];
                    double targetW = producedWeigths[j];
                    if (lockJoints[j] == 0) {  // unlock
                        targetW *= mult;       // normalement divide par 1, sauf cas lock joints
                        theWeights[i * nbJoints + j] = targetW;
                    } else {
                        theWeights[i * nbJoints + j] = currentW;
                    }
                }
            } else {
                for (unsigned int j = 0; j < nbJoints; ++j) {
                    theWeights[i * nbJoints + j] = fullWeightArray[theVert * nbJoints + j];
                }
            }
            i++;
        }
    } else {
        // do the command --------------------------
        int i = -1;  // i is a short index instead of theVert
        if (verbose)
            MGlobal::displayInfo(MString("-> editArray | valuesToSet ") + valuesToSet.size());
        if (verbose) MGlobal::displayInfo(MString("-> editArray | mutliplier ") + mutliplier);
        for (const auto& elem : valuesToSet) {
            i++;
            int theVert = elem.first;
            double theVal = mutliplier * elem.second;
            // get the sum of weights
            if (verbose)
                MGlobal::displayInfo(MString("-> editArray | theVert ") + theVert +
                                     MString(" | i ") + i + MString(" | theVal ") + theVal);

            double sumUnlockWeights = 0.0;
            for (int jnt = 0; jnt < nbJoints; ++jnt) {
                int indexArray_theWeight = i * nbJoints + jnt;
                int indexArray_fullWeightArray = theVert * nbJoints + jnt;

                // if (verbose) MGlobal::displayInfo(MString("-> editArray | jnt ") + jnt +
                // MString("-> editArray | indexArray_theWeight ") + indexArray_theWeight); if
                // (verbose) MGlobal::displayInfo(MString("-> editArray | indexArray_fullWeightArray
                // ") + indexArray_fullWeightArray);

                if (indexArray_theWeight > theWeights.length()) {
                    MGlobal::displayInfo(
                        MString(
                            "-> editArray FAILED | indexArray_theWeight  > theWeights.length()") +
                        indexArray_theWeight + MString(" > ") + theWeights.length());
                    return MStatus::kFailure;
                }
                if (indexArray_fullWeightArray > fullWeightArray.length()) {
                    MGlobal::displayInfo(MString("-> editArray FAILED | indexArray_fullWeightArray "
                                                 " > fullWeightArray.length()") +
                                         indexArray_fullWeightArray + MString(" > ") +
                                         fullWeightArray.length());
                    return MStatus::kFailure;
                }

                // if (verbose) MGlobal::displayInfo(MString("-> editArray | lockJoints[") + jnt+
                // MString("] : ") + lockJoints[jnt]);
                if (lockJoints[jnt] == 0) {  // not locked
                    sumUnlockWeights += fullWeightArray[indexArray_fullWeightArray];
                }
                theWeights[indexArray_theWeight] =
                    fullWeightArray[indexArray_fullWeightArray];  // preset array
            }
            if (verbose) MGlobal::displayInfo(MString("-> editArray | AFTER joints  loop"));
            double currentW = fullWeightArray[theVert * nbJoints + influence];

            if (((command == ModifierCommands::Remove) || (command == ModifierCommands::Absolute)) &&
                (currentW > (sumUnlockWeights - .0001))) {  // value is 1(max) we cant do anything
                continue;                                   // we pass to next vertex
            }

            double newW = currentW;
            if (command == ModifierCommands::Add)
                newW += theVal;
            else if (command == ModifierCommands::Remove)
                newW -= theVal;
            else if (command == ModifierCommands::AddPercent)
                newW += theVal * newW;
            else if (command == ModifierCommands::Absolute)
                newW = theVal;

            newW = std::max(0.0, std::min(newW, sumUnlockWeights));  // clamp

            double newRest = sumUnlockWeights - newW;
            double oldRest = sumUnlockWeights - currentW;
            double div = sumUnlockWeights;

            if (newRest != 0.0) div = oldRest / newRest;  // produit en croix

            // do the locks !!
            double sum = 0.0;
            for (int jnt = 0; jnt < nbJoints; ++jnt) {
                if (lockJoints[jnt] == 1) {
                    continue;
                }
                // check the zero val ----------
                double weightValue = fullWeightArray[theVert * nbJoints + jnt];
                if (jnt == influence) {
                    weightValue = newW;
                } else {
                    if (newW == sumUnlockWeights) {
                        weightValue = 0.0;
                    } else {
                        weightValue /= div;
                    }
                }
                if (normalize) {
                    weightValue = std::max(0.0, std::min(weightValue, sumUnlockWeights));  // clamp
                }
                sum += weightValue;
                theWeights[i * nbJoints + jnt] = weightValue;
            }

            if ((sum == 0) ||
                (sum <
                 0.5 * sumUnlockWeights)) {  // zero problem revert weights ----------------------
                for (int jnt = 0; jnt < nbJoints; ++jnt) {
                    theWeights[i * nbJoints + jnt] = fullWeightArray[theVert * nbJoints + jnt];
                }
            } else if (normalize && (sum != sumUnlockWeights)) {  // normalize ---------------
                for (int jnt = 0; jnt < nbJoints; ++jnt)
                    if (lockJoints[jnt] == 0) {
                        theWeights[i * nbJoints + jnt] /= sum;               // to 1
                        theWeights[i * nbJoints + jnt] *= sumUnlockWeights;  // to sum weights
                    }
            }
        }
    }
    return stat;
}

MStatus editArrayMirror(ModifierCommands command, int influence, int influenceMirror, int nbJoints,
                        MIntArray& lockJoints, MDoubleArray& fullWeightArray,
                        std::map<int, std::pair<float, float>>& valuesToSetMirror,
                        MDoubleArray& theWeights, bool normalize, double mutliplier, bool verbose) {
    MStatus stat;
    // 0 Add - 1 Remove - 2 AddPercent - 3 Absolute - 4 Smooth - 5 Sharpen - 6 LockVertices - 7
    // UnLockVertices
    //
    if (lockJoints.length() < nbJoints) {
        MGlobal::displayInfo(MString("-> editArrayMirror FAILED | nbJoints ") + nbJoints +
                             MString(" | lockJoints ") + lockJoints.length());
        return MStatus::kFailure;
    }
    if (verbose)
        MGlobal::displayInfo(MString("-> editArrayMirror | theWeights ") + theWeights.length() +
                             MString(" | fullWeightArray ") + fullWeightArray.length());
    if (command == ModifierCommands::Sharpen) {
        int i = 0;
        for (const auto& elem : valuesToSetMirror) {
            int theVert = elem.first;
            float valueBase = elem.second.first;
            float valueMirror = elem.second.second;

            float sumValue = std::min(float(1.0), valueBase + valueMirror);
            float biggestValue = std::max(valueBase, valueMirror);

            double theVal = mutliplier * (double)biggestValue + 1.0;
            double substract = theVal / nbJoints;

            MDoubleArray producedWeigths(nbJoints, 0.0);
            double totalBaseVtxUnlock = 0.0, totalBaseVtxLock = 0.0;
            ;
            double totalVtxUnlock = 0.0, totalVtxLock = 0.0;
            for (int j = 0; j < nbJoints; ++j) {
                double currentW = fullWeightArray[theVert * nbJoints + j];
                double targetW = (currentW * theVal) - substract;
                targetW = std::max(0.0, std::min(targetW, 1.0));  // clamp
                producedWeigths.set(targetW, j);
                if (lockJoints[j] == 0) {  // unlock
                    totalBaseVtxUnlock += currentW;
                    totalVtxUnlock += targetW;
                } else {
                    totalBaseVtxLock += currentW;
                    totalVtxLock += targetW;
                }
            }
            // now normalize
            double normalizedValueAvailable = 1.0 - totalBaseVtxLock;
            if (normalizedValueAvailable > 0.0 &&
                totalVtxUnlock > 0.0) {  // we have room to set weights
                double mult = normalizedValueAvailable / totalVtxUnlock;
                for (unsigned int j = 0; j < nbJoints; ++j) {
                    double currentW = fullWeightArray[theVert * nbJoints + j];
                    double targetW = producedWeigths[j];
                    if (lockJoints[j] == 0) {  // unlock
                        targetW *= mult;       // normalement divide par 1, sauf cas lock joints
                        theWeights[i * nbJoints + j] = targetW;
                    } else {
                        theWeights[i * nbJoints + j] = currentW;
                    }
                }
            } else {
                for (unsigned int j = 0; j < nbJoints; ++j) {
                    theWeights[i * nbJoints + j] = fullWeightArray[theVert * nbJoints + j];
                }
            }
            i++;
        }
    } else {
        // do the other command --------------------------
        int i = -1;  // i is a short index instead of theVert
        if (verbose)
            MGlobal::displayInfo(MString("-> editArrayMirror | valuesToSet ") +
                                 valuesToSetMirror.size());
        if (verbose) MGlobal::displayInfo(MString("-> editArrayMirror | mutliplier ") + mutliplier);
        for (const auto& elem : valuesToSetMirror) {
            i++;
            int theVert = elem.first;
            double valueBase = mutliplier * (double)elem.second.first;
            double valueMirror = mutliplier * (double)elem.second.second;

            if (influenceMirror == influence) {
                valueBase = std::max(valueBase, valueMirror);
                valueMirror = 0.0;
            }

            double sumUnlockWeights = 0.0;
            for (int jnt = 0; jnt < nbJoints; ++jnt) {
                int indexArray_theWeight = i * nbJoints + jnt;
                int indexArray_fullWeightArray = theVert * nbJoints + jnt;
                if (lockJoints[jnt] == 0) {  // not locked
                    sumUnlockWeights += fullWeightArray[indexArray_fullWeightArray];
                }
                theWeights[indexArray_theWeight] =
                    fullWeightArray[indexArray_fullWeightArray];  // preset array
            }

            if (verbose) MGlobal::displayInfo(MString("-> editArrayMirror | AFTER joints  loop"));
            double currentW = fullWeightArray[theVert * nbJoints + influence];
            double currentWMirror = fullWeightArray[theVert * nbJoints + influenceMirror];
            // 1 Remove 3 Absolute
            double newW = currentW;
            double newWMirror = currentWMirror;
            double sumNewWs = newW + newWMirror;

            if (command == ModifierCommands::Add) {
                newW = std::min(1.0, newW + valueBase);
                newWMirror = std::min(1.0, newWMirror + valueMirror);
                sumNewWs = newW + newWMirror;

                if (sumNewWs > 1.0) {
                    newW /= sumNewWs;
                    newWMirror /= sumNewWs;
                }
            } else if (command == ModifierCommands::Remove) {
                newW = std::max(0.0, newW - valueBase);
                newWMirror = std::max(0.0, newWMirror - valueMirror);
            } else if (command == ModifierCommands::AddPercent) {
                newW += valueBase * newW;
                newW = std::min(1.0, newW);
                newWMirror += valueMirror * newWMirror;
                newWMirror = std::min(1.0, newWMirror);
                sumNewWs = newW + newWMirror;
                if (sumNewWs > 1.0) {
                    newW /= sumNewWs;
                    newWMirror /= sumNewWs;
                }
            } else if (command == ModifierCommands::Absolute) {
                newW = valueBase;
                newWMirror = valueMirror;
            }
            newW = std::min(newW, sumUnlockWeights);              // clamp to max sumUnlockWeights
            newWMirror = std::min(newWMirror, sumUnlockWeights);  // clamp to max sumUnlockWeights

            double newRest = sumUnlockWeights - newW - newWMirror;
            double oldRest = sumUnlockWeights - currentW - currentWMirror;
            double div = sumUnlockWeights;

            if (newRest != 0.0) {  // produit en croix
                div = oldRest / newRest;
            }
            // do the locks !!
            double sum = 0.0;
            for (int jnt = 0; jnt < nbJoints; ++jnt) {
                if (lockJoints[jnt] == 1) {
                    continue;
                }
                // check the zero val ----------
                double weightValue = fullWeightArray[theVert * nbJoints + jnt];
                if (jnt == influence) {
                    weightValue = newW;
                } else if (jnt == influenceMirror) {
                    weightValue = newWMirror;
                } else {
                    if ((newW + newWMirror) == sumUnlockWeights) {
                        weightValue = 0.0;
                    } else {
                        weightValue /= div;
                    }
                }
                if (normalize) {
                    weightValue = std::max(0.0, std::min(weightValue, sumUnlockWeights));  // clamp
                }
                sum += weightValue;
                theWeights[i * nbJoints + jnt] = weightValue;
            }
            if ((sum == 0) || (sum < 0.5 * sumUnlockWeights)) {  // zero problem revert weights
                for (int jnt = 0; jnt < nbJoints; ++jnt) {
                    theWeights[i * nbJoints + jnt] = fullWeightArray[theVert * nbJoints + jnt];
                }
            } else if (normalize && (sum != sumUnlockWeights)) {  // normalize
                for (int jnt = 0; jnt < nbJoints; ++jnt)
                    if (lockJoints[jnt] == 0) {
                        theWeights[i * nbJoints + jnt] /= sum;               // to 1
                        theWeights[i * nbJoints + jnt] *= sumUnlockWeights;  // to sum weights
                    }
            }
        }
    }
    return stat;
}

MStatus setAverageWeight(std::vector<int>& verticesAround, int currentVertex, int indexCurrVert,
                         int nbJoints, MIntArray& lockJoints, MDoubleArray& fullWeightArray,
                         MDoubleArray& theWeights, double strengthVal) {
    MStatus stat;
    int sizeVertices = verticesAround.size();
    unsigned int i, jnt, posi;
    // MGlobal::displayInfo(MString(" paint smooth vtx [")+ currentVertex+ MString("] index - ") +
    // indexCurrVert + MString(" aroundCount ") + sizeVertices);

    MDoubleArray sumWeigths(nbJoints, 0.0);
    // compute sum weights
    for (int vertIndex : verticesAround) {
        for (jnt = 0; jnt < nbJoints; jnt++) {
            posi = vertIndex * nbJoints + jnt;
            sumWeigths[jnt] += fullWeightArray[posi];
        }
    }
    double totalBaseVtxUnlock = 0.0, totalBaseVtxLock = 0.0;
    ;
    double totalVtxUnlock = 0.0, totalVtxLock = 0.0;

    for (jnt = 0; jnt < nbJoints; jnt++) {
        // get if jnt is locked
        bool isLockJnt = lockJoints[jnt] == 1;
        int posi = currentVertex * nbJoints + jnt;
        // get currentWeight of currentVtx
        double currentW = fullWeightArray[posi];

        sumWeigths[jnt] /= sizeVertices;
        sumWeigths[jnt] = strengthVal * sumWeigths[jnt] + (1.0 - strengthVal) * currentW;  // add with strength
        double targetW = sumWeigths[jnt];

        // sum it all
        if (!isLockJnt) {
            totalBaseVtxUnlock += currentW;
            totalVtxUnlock += targetW;
        } else {
            totalBaseVtxLock += currentW;
            totalVtxLock += targetW;
        }
    }
    // setting part ---------------
    double normalizedValueAvailable = 1.0 - totalBaseVtxLock;

    if (normalizedValueAvailable > 0.0 && totalVtxUnlock > 0.0) {  // we have room to set weights
        double mult = normalizedValueAvailable / totalVtxUnlock;
        for (jnt = 0; jnt < nbJoints; jnt++) {
            bool isLockJnt = lockJoints[jnt] == 1;
            int posiToSet = indexCurrVert * nbJoints + jnt;
            int posi = currentVertex * nbJoints + jnt;

            double currentW = fullWeightArray[posi];
            double targetW = sumWeigths[jnt];

            if (isLockJnt) {
                theWeights[posiToSet] = currentW;
            } else {
                targetW *= mult;  // normalement divide par 1, sauf cas lock joints
                theWeights[posiToSet] = targetW;
            }
        }
    } else {  // normalize problem let's revert
        for (jnt = 0; jnt < nbJoints; jnt++) {
            int posiToSet = indexCurrVert * nbJoints + jnt;
            int posi = currentVertex * nbJoints + jnt;

            double currentW = fullWeightArray[posi];
            theWeights[posiToSet] = currentW;  // set the base Weight
        }
    }
    return MS::kSuccess;
}

void lineC(short x0, short y0, short x1, short y1, std::vector<std::pair<short, short>>& posi) {
    short dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    short dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    short err = (dx > dy ? dx : -dy) / 2, e2;

    for (;;) {
        // setPixel(x0, y0);
        posi.push_back(std::make_pair(x0, y0));

        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dy) {
            err += dx;
            y0 += sy;
        }
    }
}

bool RayIntersectsBBox(MPoint minPt, MPoint maxPt, MPoint orig, MVector direction) {
    double tmin = (minPt.x - orig.x) / direction.x;
    double tmax = (maxPt.x - orig.x) / direction.x;
    double tmpSwap;

    if (tmin > tmax) {
        tmpSwap = tmin;
        tmin = tmax;
        tmax = tmpSwap;
    }

    double tymin = (minPt.y - orig.y) / direction.y;
    double tymax = (maxPt.y - orig.y) / direction.y;

    if (tymin > tymax) {
        tmpSwap = tymin;
        tymin = tymax;
        tymax = tmpSwap;
    }

    if ((tmin > tymax) || (tymin > tmax)) return false;

    if (tymin > tmin) tmin = tymin;

    if (tymax < tmax) tmax = tymax;

    double tzmin = (minPt.z - orig.z) / direction.z;
    double tzmax = (maxPt.z - orig.z) / direction.z;

    if (tzmin > tzmax) {
        tmpSwap = tzmin;
        tzmin = tzmax;
        tzmax = tmpSwap;
    }

    if ((tmin > tzmax) || (tzmin > tmax)) return false;

    if (tzmin > tmin) tmin = tzmin;

    if (tzmax < tmax) tmax = tzmax;

    return true;
};

MPoint offsetIntersection(const MPoint& rayPoint, const MVector& rayVector,
                          const MVector& originNormal) {
    // A little hack to shift the input ray point around to get the intersections with the offset
    // planes
    MVector diff = rayPoint - originNormal;
    double prod = (diff * originNormal) / (rayVector * originNormal);
    return rayPoint - (rayVector * prod) + originNormal;
}

MMatrix bboxMatrix(const MPoint& minPoint, const MPoint& maxPoint, const MMatrix& bbSpace) {
    // Build the matrix of the bounding box as if it were a transformed 2x2x2 cube centered at the
    // origin
    MPoint c = (minPoint + maxPoint) / 2.0;
    MVector s = maxPoint - c;
    double matVals[4][4] = {
        {s.x, 0.0, 0.0, 0.0}, {0.0, s.y, 0.0, 0.0}, {0.0, 0.0, s.z, 0.0}, {c.x, c.y, c.z, 1.0}};
    return bbSpace * MMatrix(matVals);
}

inline bool inUnitPlane(const MPoint& inter) {
    // Quickly check if the intersection happened in the unit plane
    return (inter.x <= 1.0 && inter.x >= -1.0) && (inter.y <= 1.0 && inter.y >= -1.0) &&
           (inter.z <= 1.0 && inter.z >= -1.0);
}

void getRawNeighbors(const MIntArray& counts, const MIntArray& indices, int numVerts,
                     std::vector<std::unordered_set<int>>& faceNeighbors,
                     std::vector<std::unordered_set<int>>& edgeNeigbors) {
    size_t ptr = 0;
    faceNeighbors.resize(numVerts);
    edgeNeigbors.resize(numVerts);
    for (const int& c : counts) {
        for (int i = 0; i < c; ++i) {
            int j = (i + 1) % c;
            int rgt = indices[ptr + i];
            int lft = indices[ptr + j];
            edgeNeigbors[rgt].insert(lft);
            edgeNeigbors[lft].insert(rgt);
            for (int x = 0; x < c; ++x) {
                if (x == i) continue;
                faceNeighbors[lft].insert(indices[ptr + x]);
            }
        }
        ptr += c;
    }
}

void convertToCountIndex(const std::vector<std::unordered_set<int>>& input,
                         std::vector<int>& counts, std::vector<int>& indices) {
    // Convert to the flattened vector/vector for usage.
    // This can have faster access later because it uses contiguous memory
    counts.push_back(0);
    for (auto& uSet : input) {
        counts.push_back(counts.back() + uSet.size());
        indices.insert(indices.end(), uSet.begin(), uSet.end());
    }
}
