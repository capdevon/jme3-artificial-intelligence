package com.jme3.ai.navmesh.gen;

import java.io.IOException;

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;

/**
 *
 * @author capdevon
 */
public class NavMeshBuildSettings implements Savable {
    
    protected float cellSize = 1f;
    protected float cellHeight = 1.5f;
    protected float minTraversableHeight = 7.5f;
    protected float maxTraversableStep = 1f;
    protected float maxTraversableSlope = 48.0f;
    protected boolean clipLedges = false;
    protected float traversableAreaBorderSize = 1.2f;
    protected int smoothingThreshold = 2;
    protected boolean useConservativeExpansion = true;
    protected int minUnconnectedRegionSize = 3;
    protected int mergeRegionSize = 10;
    protected float maxEdgeLength = 0;
    protected float edgeMaxDeviation = 2.4f;
    protected int maxVertsPerPoly = 6;
    protected float contourSampleDistance = 25;
    protected float contourMaxDeviation = 25;
    
    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule oc = ex.getCapsule(this);
        oc.write(cellSize,                  "cellSize", 1f);
        oc.write(cellHeight,                "cellHeight", 1.5f);
        oc.write(minTraversableHeight,      "minTraversableHeight", 7.5f);
        oc.write(maxTraversableStep,        "maxTraversableStep", 1f);
        oc.write(maxTraversableSlope,       "maxTraversableSlope", 48f);
        oc.write(clipLedges,                "clipLedges", false);
        oc.write(traversableAreaBorderSize, "traversableAreaBorderSize", 1.2f);
        oc.write(smoothingThreshold,        "smoothingThreshold", 2);
        oc.write(useConservativeExpansion,  "useConservativeExpansion", true);
        oc.write(minUnconnectedRegionSize,  "minUnconnectedRegionSize", 3);
        oc.write(mergeRegionSize,           "mergeRegionSize", 10);
        oc.write(maxEdgeLength,             "maxEdgeLength", 0);
        oc.write(edgeMaxDeviation,          "edgeMaxDeviation", 2.4f);
        oc.write(maxVertsPerPoly,           "maxVertsPerPoly", 6);
        oc.write(contourSampleDistance,     "contourSampleDistance", 25);
        oc.write(contourMaxDeviation,       "contourMaxDeviation", 25);
    }

    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule ic = im.getCapsule(this);
        cellSize                    = ic.readFloat("cellSize", 1f);
        cellHeight                  = ic.readFloat("cellHeight", 1.5f);
        minTraversableHeight        = ic.readFloat("minTraversableHeight", 7.5f);
        maxTraversableStep          = ic.readFloat("maxTraversableStep", 1f);
        maxTraversableSlope         = ic.readFloat("maxTraversableSlope", 48f);
        clipLedges                  = ic.readBoolean("clipLedges", false);
        traversableAreaBorderSize   = ic.readFloat("traversableAreaBorderSize", 1.2f);
        smoothingThreshold          = ic.readInt("smoothingThreshold", 2);
        useConservativeExpansion    = ic.readBoolean("useConservativeExpansion", true);
        minUnconnectedRegionSize    = ic.readInt("minUnconnectedRegionSize", 3);
        mergeRegionSize             = ic.readInt("mergeRegionSize", 10);
        maxEdgeLength               = ic.readFloat("maxEdgeLength", 0);
        edgeMaxDeviation            = ic.readFloat("edgeMaxDeviation", 2.4f);
        maxVertsPerPoly             = ic.readInt("maxVertsPerPoly", 6);
        contourSampleDistance       = ic.readFloat("contourSampleDistance", 25);
        contourMaxDeviation         = ic.readFloat("contourMaxDeviation", 25);
    }
    
	/**
     * @return The height resolution used when sampling the source mesh. Value
     * must be > 0.
     */
    public float getCellHeight() {
        return cellHeight;
    }

    /**
     * @param cellHeight - The height resolution used when sampling the source
     * mesh. Value must be > 0. Constraints: > 0
     */
    public void setCellHeight(float cellHeight) {
        this.cellHeight = cellHeight;
        if (this.cellHeight <= 0) {
            this.cellHeight = 0.001f;
        }
    }

    /**
     * @return The width and depth resolution used when sampling the the source
     * mesh.
     */
    public float getCellSize() {
        return cellSize;
    }

    /**
     * @param cellSize - The width and depth resolution used when sampling the
     * the source mesh. Constraints: > 0
     */
    public void setCellSize(float cellSize) {
        this.cellSize = cellSize;
        if (this.cellSize <= 0) {
            this.cellSize = 0.001f;
        }
    }

    /**
     *
     * @return Indicates whether ledges are considered un-walkable
     */
    public boolean isClipLedges() {
        return clipLedges;
    }

    /**
     * Indicates whether ledges should be marked as unwalkable. A ledge is a
     * normally walkable voxel that has one or more accessible neighbors with a
     * an un-steppable drop from voxel top to voxel top. E.g. If an agent using
     * the navmesh were to travel down from the ledge voxel to its neighbor
     * voxel, it would result in the maximum traversable step distance being
     * violated. The agent cannot legally "step down" from a ledge to its
     * neighbor.
     *
     * @param clipLedges
     */
    public void setClipLedges(boolean clipLedges) {
        this.clipLedges = clipLedges;
    }

    /**
     *
     * @return the contour max deviation
     */
    public float getContourMaxDeviation() {
        return contourMaxDeviation;
    }

    /**
     * The maximum distance the surface of the navmesh may deviate from the
     * surface of the original geometry. The accuracy of the algorithm which
     * uses this value is impacted by the value of the contour sample distance
     * argument. The value of this argument has no meaning if the contour sample
     * distance argument is set to zero. Setting the value to zero is not
     * recommended since it can result in a large increase in the number of
     * triangles in the final navmesh at a high processing cost. Constraints: >=
     * 0
     *
     * @param contourMaxDeviation
     */
    public void setContourMaxDeviation(float contourMaxDeviation) {
        this.contourMaxDeviation = contourMaxDeviation;
        if (this.contourMaxDeviation < 0) {
            this.contourMaxDeviation = 0.0f;
        }
    }

    /**
     *
     * @return the contour sample distance
     */
    public float getContourSampleDistance() {
        return contourSampleDistance;
    }

    /**
     * Sets the sampling distance to use when matching the navmesh to the
     * surface of the original geometry. Impacts how well the final mesh
     * conforms to the original geometry's surface contour. Higher values result
     * in a navmesh which conforms more closely to the original geometry's
     * surface at the cost of a higher final triangle count and higher
     * processing cost. Setting this argument to zero will disable this
     * functionality. Constraints: >= 0
     *
     * @param contourSampleDistance
     */
    public void setContourSampleDistance(float contourSampleDistance) {
        this.contourSampleDistance = contourSampleDistance;
        if (this.contourSampleDistance < 0) {
            this.contourSampleDistance = 0;
        }
    }

    /**
     *
     * @return the edge max deviation
     */
    public float getEdgeMaxDeviation() {
        return edgeMaxDeviation;
    }

    /**
     * The maximum distance the edge of the navmesh may deviate from the source
     * geometry. Setting this lower will result in the navmesh edges following
     * the geometry contour more accurately at the expense of an increased
     * triangle count. Setting the value to zero is not recommended since it can
     * result in a large increase in the number of triangles in the final
     * navmesh at a high processing cost. Constraints: >= 0
     *
     * @param edgeMaxDeviation
     */
    public void setEdgeMaxDeviation(float edgeMaxDeviation) {
        this.edgeMaxDeviation = edgeMaxDeviation;
        if (this.edgeMaxDeviation < 0) {
            this.edgeMaxDeviation = 0;
        }
    }

    /**
     *
     * @return the edge max length
     */
    public float getMaxEdgeLength() {
        return maxEdgeLength;
    }

    /**
     * The maximum length of polygon edges that represent the border of the
     * navmesh. More vertices will be added to navmesh border edges if this
     * value is exceeded for a particular edge. In certain cases this will
     * reduce the number of thin, long triangles in the navmesh. A value of zero
     * will disable this feature. Constraints: >= 0
     *
     * @param maxEdgeLength
     */
    public void setMaxEdgeLength(float maxEdgeLength) {
        this.maxEdgeLength = maxEdgeLength;
        if (this.maxEdgeLength < 0) {
            this.maxEdgeLength = 0;
        }
    }

    /**
     *
     * @return the maximum traversable slope in degrees
     */
    public float getMaxTraversableSlope() {
        return maxTraversableSlope;
    }

    /**
     * The maximum slope that is considered walkable. (Degrees) Constraints: 0
     * <= value <= 85
     *
     * @param maxTraversableSlope
     */
    public void setMaxTraversableSlope(float maxTraversableSlope) {
        this.maxTraversableSlope = maxTraversableSlope;
        if (this.maxTraversableSlope < 0) {
            this.maxTraversableSlope = 0;
        }
        if (this.maxTraversableSlope > 85) {
            this.maxTraversableSlope = 85;
        }
    }

    /**
     *
     * @return the maximum traversable step
     */
    public float getMaxTraversableStep() {
        return maxTraversableStep;
    }

    /**
     * Represents the maximum ledge height that is considered to still be
     * walkable. Prevents minor deviations in height from improperly showing as
     * obstructions. Permits detection of stair-like structures, curbs, etc.
     * Constraints: >= 0
     *
     * @param maxTraversableStep
     */
    public void setMaxTraversableStep(float maxTraversableStep) {
        this.maxTraversableStep = maxTraversableStep;
        if (this.maxTraversableStep < 0) {
            this.maxTraversableStep = 0;
        }
    }

    /**
     *
     * @return the maximum vertices per polygon
     */
    public int getMaxVertsPerPoly() {
        return maxVertsPerPoly;
    }

    /**
     * The maximum number of vertices per polygon for polygons generated during
     * the voxel to polygon conversion stage. Higher values reduce performance,
     * but can also result in better formed triangles in the navmesh. A value of
     * around 6 is generally adequate with diminishing returns for values higher
     * than 6. Contraints: >= 3
     *
     * @param maxVertsPerPoly
     */
    public void setMaxVertsPerPoly(int maxVertsPerPoly) {
        this.maxVertsPerPoly = maxVertsPerPoly;
        if (this.maxVertsPerPoly < 3) {
            this.maxVertsPerPoly = 3;
        }
    }

    /**
     *
     * @return the merge region size
     */
    public int getMergeRegionSize() {
        return mergeRegionSize;
    }

    /**
     * Any regions smaller than this size will, if possible, be merged with
     * larger regions. (Voxels) Helps reduce the number of unnecessarily small
     * regions that can be formed. This is especially an issue in diagonal path
     * regions where inherent faults in the region generation algorithm can
     * result in unnecessarily small regions. If a region cannot be legally
     * merged with a neighbor region, then it will be left alone. Constraints:
     * >= 0
     *
     * @param mergeRegionSize
     */
    public void setMergeRegionSize(int mergeRegionSize) {
        this.mergeRegionSize = mergeRegionSize;
        if (this.mergeRegionSize < 0) {
            this.mergeRegionSize = 0;
        }
    }

    /**
     *
     * @return the minimum traversable height
     */
    public float getMinTraversableHeight() {
        return minTraversableHeight;
    }

    /**
     * Represents the minimum floor to ceiling height that will still allow the
     * floor area to be considered walkable. Permits detection of overhangs in
     * the geometry which make the geometry below become unwalkable.
     * Constraints: > 0
     *
     * @param minTraversableHeight
     */
    public void setMinTraversableHeight(float minTraversableHeight) {
        this.minTraversableHeight = minTraversableHeight;
        if (this.minTraversableHeight <= 0) {
            this.minTraversableHeight = 0.001f;
        }
    }

    /**
     *
     * @return the minimum unconnected region size
     */
    public int getMinUnconnectedRegionSize() {
        return minUnconnectedRegionSize;
    }

    /**
     * The minimum region size for unconnected (island) regions. (Voxels) Any
     * generated regions that are not connected to any other region and are
     * smaller than this size will be culled before final navmesh generation.
     * I.e. No longer considered walkable. Constraints: > 0
     *
     * @param minUnconnectedRegionSize
     */
    public void setMinUnconnectedRegionSize(int minUnconnectedRegionSize) {
        this.minUnconnectedRegionSize = minUnconnectedRegionSize;
        if (this.minUnconnectedRegionSize <= 0) {
            this.minUnconnectedRegionSize = 1;
        }
    }

    /**
     *
     * @return the smoothing threshold
     */
    public int getSmoothingThreshold() {
        return smoothingThreshold;
    }

    /**
     * The amount of smoothing to be performed when generating the distance
     * field. This value impacts region formation and border detection. A higher
     * value results in generally larger regions and larger border sizes. A
     * value of zero will disable smoothing. Constraints: 0 <= value <= 4
     *
     * @param smoothingThreshold
     */
    public void setSmoothingThreshold(int smoothingThreshold) {
        this.smoothingThreshold = smoothingThreshold;
        if (this.smoothingThreshold < 0) {
            this.smoothingThreshold = 0;
        }
        if (this.smoothingThreshold > 4) {
            this.smoothingThreshold = 4;
        }
    }

    /**
     *
     * @return the traversable border size
     */
    public float getTraversableAreaBorderSize() {
        return traversableAreaBorderSize;
    }

    /**
     * Represents the closest any part of the navmesh can get to an obstruction
     * in the source mesh. Usually set to the maximum bounding radius of
     * entities utilizing the navmesh for navigation decisions. Constraints: >=
     * 0
     *
     * @param traversableAreaBorderSize
     */
    public void setTraversableAreaBorderSize(float traversableAreaBorderSize) {
        this.traversableAreaBorderSize = traversableAreaBorderSize;
        if (this.traversableAreaBorderSize < 0) {
            this.traversableAreaBorderSize = 0;
        }
    }

    /**
     *
     * @return whether or not using conservative expansion
     */
    public boolean isUseConservativeExpansion() {
        return useConservativeExpansion;
    }

    /**
     * Applies extra algorithms to regions to help prevent poorly formed regions
     * from forming. If the navigation mesh is missing sections that should be
     * present, then enabling this feature will likely fix the problem. Enabling
     * this feature significantly increased processing cost.
     *
     * @param useConservativeExpansion
     */
    public void setUseConservativeExpansion(boolean useConservativeExpansion) {
        this.useConservativeExpansion = useConservativeExpansion;
    }
}
