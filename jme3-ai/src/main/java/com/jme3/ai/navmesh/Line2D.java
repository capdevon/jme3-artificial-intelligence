package com.jme3.ai.navmesh;

import com.jme3.export.InputCapsule;
import com.jme3.export.JmeExporter;
import com.jme3.export.JmeImporter;
import com.jme3.export.OutputCapsule;
import com.jme3.export.Savable;
import java.io.IOException;

import com.jme3.math.Vector2f;

/**
 * 
 * Line2D represents a line in 2D space. Line data is held as a line segment
 * having two endpoints and as a fictional 3D plane extending vertically. The
 * Plane is then used for spanning and point classification tests. A Normal
 * vector is used internally to represent the fictional plane.
 * 
 * Portions Copyright (C) Greg Snook, 2000
 * 
 * @author TR
 *
 */
public class Line2D implements Savable {

    enum PointSide {
        /**
         *  The point is on, or very near, the line
         */
        OnLine,
        /**
         * looking from endpoint A to B, the test point is on the left
         */
        Left,
        /**
         * looking from endpoint A to B, the test point is on the right
         */
        Right;
    }

    enum LineIntersect {
        /**
         * both lines are parallel and overlap each other
         */
        CoLinear,
        /**
         *  lines intersect, but their segments do not
         */
        LinesIntersect,
        /**
         * both line segments bisect each other
         */
        SegmentsIntersect,
        /**
         * line segment B is crossed by line A
         */
        ABisectsB,
        /**
         *  line segment A is crossed by line B
         */
        BBisectsA,
        /**
         * the lines are parallel
         */
        Parallel;
    }
    /**
     * Endpoint A of our line segment
     */
    private Vector2f pointA;
    /**
     * Endpoint B of our line segment
     */
    private Vector2f pointB;
    /**
     * 'normal' of the ray.
     * a vector pointing to the right-hand side of the line
     * when viewed from PointA towards PointB
     */
    private volatile Vector2f normal;

    public Line2D(Vector2f pointA, Vector2f pointB) {
        this.pointA = pointA;
        this.pointB = pointB;
        normal = null;
    }

    public void setPointA(Vector2f point) {
        this.pointA = point;
        normal = null;
    }

    public void setPointB(Vector2f point) {
        this.pointB = point;
        normal = null;
    }

    public void setPoints(Vector2f pointA, Vector2f pointB) {
        this.pointA = pointA;
        this.pointB = pointB;
        normal = null;
    }

    public Vector2f getNormal() {
        if (normal == null)
            computeNormal();
        
        return normal;
    }

    public void setPoints(float pointAx, float pointAy, float pointBx, float pointBy) {
        pointA.x = pointAx;
        pointA.y = pointAy;
        pointB.x = pointBx;
        pointB.y = pointBy;
        normal = null;
    }

    public Vector2f getPointA() {
        return pointA;
    }

    public Vector2f getPointB() {
        return pointB;
    }

    public float length() {
        float xdist = pointB.x - pointA.x;
        float ydist = pointB.y - pointA.y;

        xdist *= xdist;
        ydist *= ydist;

        return (float) Math.sqrt(xdist + ydist);
    }

    public Vector2f getDirection() {
        return pointB.subtract(pointA).normalizeLocal();
    }

    private void computeNormal() {
        // Get Normalized direction from A to B
        normal = getDirection();

        // Rotate by -90 degrees to get normal of line
        float oldY = normal.y;
        normal.y = -normal.x;
        normal.x = oldY;
    }

    /**
     * Determines the signed distance from a point to this line. Consider the line
     * as if you were standing on PointA of the line looking towards PointB.
     * Positive distances are to the right of the line, negative distances are to
     * the left.
     */
    public float signedDistance(Vector2f point) {
        if (normal == null) {
            computeNormal();
        }

        return point.subtract(pointA).dot(normal); //.x*m_Normal.x + TestVector.y*m_Normal.y;//DotProduct(TestVector,m_Normal);
    }

    /**
     * Determines where a point lies in relation to this line. Consider the line as
     * if you were standing on PointA of the line looking towards PointB. The incoming
     * point is then classified as being on the Left, Right or Centered on the line.
     */
//    public PointSide getSide(Vector2f point, float epsilon) {
//        PointSide result = PointSide.OnLine;
//        float distance = signedDistance(point);
//
//        if (distance > epsilon) {
//            result = PointSide.Right;
//        } else if (distance < -epsilon) {
//            result = PointSide.Left;
//        }
//
//        return result;
//    }

    // this works much more correctly
    public PointSide getSide(Vector2f c, float epsilon) {
        Vector2f a = pointA;
        Vector2f b = pointB;
        float res = ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x));
        if (res > 0)
            return PointSide.Left;
        else if (res == 0)
            return PointSide.OnLine;
        else 
            return PointSide.Right;
    }
    
    /**
     * this line A = x0, y0 and B = x1, y1 
     * other is A = x2, y2 and B = x3, y3
     * 
     * Determines the type of intersection between this line and another line. If
     * the lines intersect, the intersection point is stored in the provided
     * Vector2f.
     *
     * @param line  The other line to check for intersection.
     * @param store The point where the lines intersect, if they do. Can be null.
     * @return The type of intersection, as a value from the LineIntersect enum.
     */
    public LineIntersect intersect(Line2D line, Vector2f store) {
        float denom = (line.pointB.y - line.pointA.y) * (this.pointB.x - this.pointA.x)
                - (line.pointB.x - line.pointA.x) * (this.pointB.y - this.pointA.y);
        float u0 = (line.pointB.x - line.pointA.x) * (this.pointA.y - line.pointA.y)
                - (line.pointB.y - line.pointA.y) * (this.pointA.x - line.pointA.x);
        float u1 = (line.pointA.x - this.pointA.x) * (this.pointB.y - this.pointA.y)
                - (line.pointA.y - this.pointA.y) * (this.pointB.x - this.pointA.x);

        //if parallel
        if (denom == 0.0f) {
            //if collinear
            if (u0 == 0.0f && u1 == 0.0f) {
                return LineIntersect.CoLinear;
            } else {
                return LineIntersect.Parallel;
            }
        } else {
            //check if they intersect
            u0 = u0 / denom;
            u1 = u1 / denom;

            float x = this.pointA.x + u0 * (this.pointB.x - this.pointA.x);
            float y = this.pointA.y + u0 * (this.pointB.y - this.pointA.y);

            if (store != null) {
                store.x = x; //(m_PointA.x + (FactorAB * Bx_minus_Ax));
                store.y = y; //(m_PointA.y + (FactorAB * By_minus_Ay));
            }

            // now determine the type of intersection
            if ((u0 >= 0.0f) && (u0 <= 1.0f) && (u1 >= 0.0f) && (u1 <= 1.0f)) {
                return LineIntersect.SegmentsIntersect;
            } else if ((u1 >= 0.0f) && (u1 <= 1.0f)) {
                return (LineIntersect.ABisectsB);
            } else if ((u0 >= 0.0f) && (u0 <= 1.0f)) {
                return (LineIntersect.BBisectsA);
            }

            return LineIntersect.LinesIntersect;
        }
    }

    /**
     * Determines if two segments intersect, and if so the point of intersection. The current
     * member line is considered line AB and the incoming parameter is considered line CD for
     * the purpose of the utilized equations.
     *
     * A = PointA of the member line
     * B = PointB of the member line
     * C = PointA of the provided line
     * D = PointB of the provided line
     */
    @Deprecated
    public LineIntersect intersectionOLD(Line2D line, Vector2f pIntersectPoint) {
        float Ay_minus_Cy = pointA.y - line.getPointA().y;
        float Dx_minus_Cx = line.getPointB().x - line.getPointA().x;
        float Ax_minus_Cx = pointA.x - line.getPointA().x;
        float Dy_minus_Cy = line.getPointB().y - line.getPointA().y;
        float Bx_minus_Ax = pointB.x - pointA.x;
        float By_minus_Ay = pointB.y - pointA.y;

        java.awt.geom.Line2D l1 = new java.awt.geom.Line2D.Float(this.pointA.x, this.pointA.y, this.pointB.x, this.pointB.y);
        if (l1.intersectsLine(line.getPointA().x, line.getPointA().y, line.getPointB().x, line.getPointB().y)) //return LINE_CLASSIFICATION.LINES_INTERSECT;
        {
            System.out.println("They intersect");
        } else //return LINE_CLASSIFICATION.COLLINEAR;
        {
            System.out.println("They DO NOT intersect");
        }

        float Numerator = (Ay_minus_Cy * Dx_minus_Cx) - (Ax_minus_Cx * Dy_minus_Cy);
        float Denominator = (Bx_minus_Ax * Dy_minus_Cy) - (By_minus_Ay * Dx_minus_Cx);

        // if lines do not intersect, return now
        if (Denominator != 0.0f) {
            if (Numerator != 0.0f) {
                System.out.println("App says They DO NOT intersect");
                return LineIntersect.CoLinear;
            }
            System.out.println("App says They DO NOT intersect");
            return LineIntersect.Parallel;
        }

        float FactorAB = Numerator / Denominator;
        float FactorCD = ((Ay_minus_Cy * Bx_minus_Ax) - (Ax_minus_Cx * By_minus_Ay)) / Denominator;

        // posting (hitting a vertex exactly) is not allowed, shift the results
        // if they are within a minute range of the end vertices
//        if (fabs(FactorCD) < 1.0e-6f) {
//            FactorCD = 1.0e-6f;
//        }
//        if (fabs(FactorCD - 1.0f) < 1.0e-6f) {
//            FactorCD = 1.0f - 1.0e-6f;
//        }

        // if an intersection point was provided, fill it in now
        if (pIntersectPoint != null) {
            pIntersectPoint.x = (pointA.x + (FactorAB * Bx_minus_Ax));
            pIntersectPoint.y = (pointA.y + (FactorAB * By_minus_Ay));
        }

        System.out.println("App says They DO intersect");
        // now determine the type of intersection
        if ((FactorAB >= 0.0f) && (FactorAB <= 1.0f) && (FactorCD >= 0.0f) && (FactorCD <= 1.0f)) {
            return LineIntersect.SegmentsIntersect;
        } else if ((FactorCD >= 0.0f) && (FactorCD <= 1.0f)) {
            return (LineIntersect.ABisectsB);
        } else if ((FactorAB >= 0.0f) && (FactorAB <= 1.0f)) {
            return (LineIntersect.BBisectsA);
        }

        return LineIntersect.LinesIntersect;
    }

    @Override
    public String toString() {
        return "Line2D [pointA=" + pointA + ", pointB=" + pointB + "]";
    }

    @Override
    public void write(JmeExporter ex) throws IOException {
        OutputCapsule oc = ex.getCapsule(this);
        oc.write(pointA, "pointA", null);
        oc.write(pointB, "pointB", null);
        oc.write(normal, "normal", null);
    }

    @Override
    public void read(JmeImporter im) throws IOException {
        InputCapsule ic = im.getCapsule(this);
        pointA = (Vector2f) ic.readSavable("pointA", new Vector2f());
        pointB = (Vector2f) ic.readSavable("pointB", new Vector2f());
        normal = (Vector2f) ic.readSavable("normal", new Vector2f());
    }
    
    /**
     * 
     * @param args
     */
//    public static void main(String[] args) {
//        Line2D a = new Line2D(new Vector2f(-2, 0), new Vector2f(2, 0));
//        Line2D b = new Line2D(new Vector2f(-2, 1), new Vector2f(2, -1));
//        Line2D.LineIntersect res = a.intersect(b, null);
//        if (res == LineIntersect.CoLinear || res == LineIntersect.Parallel) {
//            System.out.println("Failed intersection verification");
//        }
//        if (a.getSide(new Vector2f(0, 1), 0.0f) != PointSide.Left) {
//            System.out.println("Failed left test");
//        }
//        if (a.getSide(new Vector2f(0, -1), 0.0f) != PointSide.Right) {
//            System.out.println("Failed right test");
//        }
//        if (a.getSide(new Vector2f(0, 0), 0.0f) != PointSide.OnLine) {
//            System.out.println("Failed on line test");
//        }
//    }
    
}
