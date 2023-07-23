package com.jme3.ai.navmesh;

/**
 * Options for NavMeshQuery.findStraightPath.
 * 
 * @author capdevon
 */
public enum StraightPathOptions {

    //Add a vertex at every polygon edge crossing where area changes.
    AreaCrossings,
    //Add a vertex at every polygon edge crossing.
    AllCrossings;

}
