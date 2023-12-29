package com.jme3.ai.navmesh;

import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.logging.Logger;

import com.jme3.ai.navmesh.Cell.ClassifyResult;
import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;

/**
 * A NavigationMesh is a collection of NavigationCells used to control object
 * movement while also providing path finding line-of-sight testing. It serves
 * as a parent to all the Actor objects which exist upon it.
 * 
 * Portions Copyright (C) Greg Snook, 2000
 * 
 * @author TR
 */
public class NavMesh implements Savable {
    
    private static final Logger logger = Logger.getLogger(NavMesh.class.getName());

    /**
     * the cells that make up this mesh
     */
    private ArrayList<Cell> cellList = new ArrayList<>();
    private Mesh mesh;

    /**
     * For serialization only. Do not use.
     */
    public NavMesh() {
    }

    /**
     * Instantiate a <code>NavMesh</code>
     * @param mesh
     */
    public NavMesh(Mesh mesh) {
        loadFromMesh(mesh);
    }

    public int getNumCells() {
        return cellList.size();
    }

    public Cell getCell(int index) {
        return cellList.get(index);
    }
    
    /**
     * Force a point to be inside the nearest cell on the mesh
     */
    public Vector3f snapPointToMesh(Vector3f point) {
        Cell cell = findClosestCell(point);
        return snapPointToCell(cell, point);
    }

    /**
     * Force a point to be inside the cell
     */
    public Vector3f snapPointToCell(Cell cell, Vector3f point) {
        if (!cell.contains(point)) {
            cell.forcePointToCellColumn(point);
        }

        cell.computeHeightOnCell(point);
        return point;
    }
    
    /**
     * Finds the nearest point based on the NavMesh within a specified range.
     * 
     * @param sourcePos   The origin of the sample query.
     * @param result      Holds the properties of the resulting location
     * @param maxDistance Sample within this distance from sourcePosition.
     * @return bool True if the nearest point is found.
     */
    public boolean samplePosition(Vector3f sourcePos, Vector3f result, float maxDistance) {
        
        result.set(Vector3f.NAN);
        boolean found = false;
        
        Cell cell = findClosestCell(sourcePos, maxDistance);
        if (cell != null) {
            //snapPointToCell(cell, sourcePos);
            cell.computeHeightOnCell(sourcePos);
            result.set(sourcePos);
            found = true;
        }
        
        return found;
    }
    
    /**
     * Find the closest cell on the mesh to the given point.
     */
    public Cell findClosestCell(Vector3f point) {
        float maxDistance = Float.MAX_VALUE;
        return findClosestCell(point, maxDistance);
    }

    /**
     * Find the closest cell on the mesh to the given point.
     * 
     * @param sourcePos
     * @param maxDistance
     * @return
     */
    public Cell findClosestCell(Vector3f sourcePos, float maxDistance) {

        Cell closestCell = null;
        float closestDistance = maxDistance;
        float closestHeight = maxDistance;
        boolean found = false;

        for (Cell cell : cellList) {
            if (cell.contains(sourcePos)) {
                float distance = Math.abs(cell.getHeightOnCell(sourcePos) - sourcePos.y);

                if (found) {
                    if (distance < closestHeight) {
                        closestCell = cell;
                        closestHeight = distance;
                    }
                } else {
                    closestCell = cell;
                    closestHeight = distance;
                    found = true;
                }
            }

            if (!found) {
                Vector2f start = new Vector2f(cell.getCenter().x, cell.getCenter().z);
                Vector2f end = new Vector2f(sourcePos.x, sourcePos.z);
                Line2D motionPath = new Line2D(start, end);

                ClassifyResult cResult = cell.classifyPathToCell(motionPath);

                if (cResult.result == Cell.PathResult.ExitingCell) {
                    Vector3f closestPoint3D = new Vector3f(cResult.intersection.x, 0.0f, cResult.intersection.y);
                    cell.computeHeightOnCell(closestPoint3D);

                    float distance = closestPoint3D.distance(sourcePos);

                    if (distance < closestDistance) {
                        closestCell = cell;
                        closestDistance = distance;
                    }
                }
            }
        }

        return closestCell;
    }
    
    /**
     * Link all the cells that are in our pool
     */
    private void linkCells() {
        for (Cell a : cellList) {
            for (Cell b : cellList) {
                if (a != b) {
                    a.checkAndLink(b, 0.001f);
                }
            }
        }
    }

    /**
     * Add a new cell, defined by the three vertices in clockwise order, to this
     * mesh.
     */
    private void addCell(Vector3f vertA, Vector3f vertB, Vector3f vertC) {
        /**
         * Some art programs can create linear polygons with two or more identical
         * vertices. This creates a polygon with no surface area, which interferes with
         * navigation mesh algorithms. Only polygons with unique vertices are allowed.
         */
        if (!vertA.equals(vertB) && !vertB.equals(vertC) && !vertC.equals(vertA)) {
            Vector3f a = vertA.clone();
            Vector3f b = vertB.clone();
            Vector3f c = vertC.clone();
            Cell newCell = new Cell(a, b, c);
            cellList.add(newCell);
        } else {
            logger.warning("Warning, incorrect face winding!");
        }
    }
    
    public void loadFromMesh(Mesh mesh) {
        
        this.mesh = mesh;
        cellList.clear();
        
        Vector3f a = new Vector3f();
        Vector3f b = new Vector3f();
        Vector3f c = new Vector3f();

        Plane up = new Plane();
        up.setPlanePoints(Vector3f.UNIT_X, Vector3f.ZERO, Vector3f.UNIT_Z);

        IndexBuffer ib = mesh.getIndexBuffer();
        FloatBuffer pb = mesh.getFloatBuffer(Type.Position);
        pb.clear();
        
        for (int i = 0; i < mesh.getTriangleCount() * 3; i += 3) {
            int i1 = ib.get(i + 0);
            int i2 = ib.get(i + 1);
            int i3 = ib.get(i + 2);
            
            BufferUtils.populateFromBuffer(a, pb, i1);
            BufferUtils.populateFromBuffer(b, pb, i2);
            BufferUtils.populateFromBuffer(c, pb, i3);

            Plane p = new Plane();
            p.setPlanePoints(a, b, c);
            if (up.pseudoDistance(p.getNormal()) <= 0.0f) {
                logger.warning("Warning, the normal of the plane faces downward!");
                continue;
            }

            addCell(a, b, c);
        }

        linkCells();
    }
    
    /**
     * Calculates and returns a simple triangulation of the current navmesh.
     * 
     * @return NavMeshTriangulation
     */
    public NavMeshTriangulation calculateTriangulation() {
        IndexBuffer ib = mesh.getIndexBuffer();
        // generate int array of indices
        int[] indices = new int[ib.size()];
        for (int i = 0; i < indices.length; i++) {
            indices[i] = ib.get(i);
        }
        
        FloatBuffer pb = mesh.getFloatBuffer(VertexBuffer.Type.Position);
        Vector3f[] vertices = BufferUtils.getVector3Array(pb);

        return new NavMeshTriangulation(indices, vertices);
    }

    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule oc = ex.getCapsule(this);
        oc.writeSavableArrayList(cellList, "cellList", null);
        oc.write(mesh, "mesh", null);
    }

    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule ic = im.getCapsule(this);
        cellList = ic.readSavableArrayList("cellList", null);
        mesh = (Mesh) ic.readSavable("mesh", null);
    }
    
}
