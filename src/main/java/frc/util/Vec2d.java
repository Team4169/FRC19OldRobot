/*
 * The MIT License
 *
 * Copyright 2018 lwa.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package frc.util;

/**
 * A 2-d mathematical vector of doubles, with the usual operations.
 * vectors are immutable.  Factory methods are provided to construct
 * a vector from cartesian and polar coordinates.
 * @author lwa
 */
public class Vec2d {
    
    /**
     * The x-coordinate
     */
    private final double m_x;
    
    /**
     * The y-coordinate
     */
    private final double m_y;
    
    /**
     * The zero vector, for convenience
     */
    public static final Vec2d ZERO = makeCart(0.0, 0.0);
    
    /**
     * A constant defining the default distance at which two vectors are
     * considered "near", for use in the "isNear" method.
     */
    public static final double EPSILON = 0.00001d;

    /**
     * Private constructor because callers use factories.
     * @param x x-coord
     * @param y y-coord
     */
     private Vec2d(double x, double y) {
         m_x = x;
         m_y = y;
     }

    /**
     * Factory method to construct a vector given its cartesian coordinates
     * @param x The x-coord
     * @param y The y-coord
     * @return the vector
     */
     public static Vec2d makeCart(double x, double y) {
         return new Vec2d(x, y);
     }

    /**
     * Factory method to construct a vector given its polar coordinates
     * @param r The r-coordinate (length)
     * @param theta The theta-coordinate (angle from X axis, in radians)
     * @return the vector
     * @throws IllegalArgumentException if the r-coordinate is negative
     */
     public static Vec2d makePolar(double r, double theta) {
         if (r < 0.0d)
             throw new IllegalArgumentException("negative vector length " + r);
         return new Vec2d(r * Math.cos(theta), r * Math.sin(theta));
     }

    /**
     * Get the x-coordinate
     * @return The x-coordinate
     */
     public double getX() {
         return m_x;
     }

    /**
     * Get the y-coordinate
     * @return The y-coordinate
     */
     public double getY() {
         return m_y;
     }

    /**
     * Get the r-coordinate
     * @return The r-coordinate
     */
     public double getR() {
         return Math.hypot(m_x, m_y);
     }

    /**
     * Get the theta-coordinate
     * @return The theta-coordinate (angle) in radians, -pi <= res <= pi
     */
     public double getTheta() {
         return Math.atan2(m_y, m_x);
     }

    /**
     * Add a vector, returning the result
     * @param a addend
     * @return The sum
     */
     public Vec2d add(Vec2d a) {
         return new Vec2d(m_x + a.m_x, m_y + a.m_y);
     }

    /**
     * Subtract a vector, returning the result
     * @param s subtrahend
     * @return The difference
     */
     public Vec2d sub(Vec2d s) {
         return new Vec2d(m_x - s.m_x, m_y - s.m_y);
     }

    /**
     * Multiply by a scalar (double), returning
     * the result.
     * @param m The scalar to multiply by
     * @return product of this vector and the scalar
     */
     public Vec2d mulScalar(double m) {
         return new Vec2d(m * m_x, m * m_y);
     }

     /**
      * Return a vector normal (perpendicular) to this
      * vector.  If v = getNormal(this), then
      *   v.sub(this).getTheta() = Math.PI / 2.0
      * that is, the returned vector is 90 deg clockwise
      * from this.  If this vector is zero, the returned
      * vector will also be zero.
      * @return A vector perpendicular to this vector, in
      * the clockwise direction (zero if this vector is zero)
      */
      public Vec2d getNormal() {
          return new Vec2d(m_y, -m_x);
      }

    /**
     * Compute the dot product of this vector with the
     * specified vector.  The dot product is a scalar,
     * defined mathematically as |a||b|cos theta 
     * where theta is the angle between a and b.
     * Computationally it's just (a.x*b.x) + (a.y*b.y)
     * @param o The other vector 
     * @return Dot product of this vector and o
     */
    public double dotProduct(Vec2d o) {
        return ((m_x * o.m_x) + (m_y * o.m_y));
    }

    /**
     * Return the additive inverse of this vector.
     * Mathematically, a + a.negate() should equal
     * the zero vector.
     * @return Additive inverse of this vector
     */
    public Vec2d negate() {
        return new Vec2d(-m_x, -m_y);
    }
     
    /**
      * Return true iff the specified vector is "near" this one.
      * "Nearness" is defined in terms of distance i.e. 
      *   abs(o - this) <= epsilon
      * In general vectors should be compared for nearness rather
      * than equality, because of floating-point rounding errors and
      * also because motion may result in inexact positioning.
      * @param o The other vector
      * @param epsilon The radius of nearness; two vectors are near if
      * their distance is less than or equal to epsilon
      * @return True iff o is near this vector
      */
     public boolean isNear(Vec2d o, double epsilon)
     {
         // Quick check for exact equality
         if (o.m_x == m_x && o.m_y == m_y)
             return true;
         
         return (Math.abs(Math.hypot(o.m_x, o.m_y) - Math.hypot(m_x, m_y)) <= epsilon);
     }
     
     /**
      * Return true iff the specified vector is "near" this one.
      * "Nearness" is defined in terms of distance i.e. 
      *   abs(o - this) <= Vec2d.EPSILON
      * In general vectors should be compared for nearness rather
      * than equality, because of floating-point rounding errors and
      * also because motion may result in inexact positioning.
      * @param o The other vector
      * @return True iff o is near this vector
      */
     public boolean isNear(Vec2d o)
     {
         return isNear(o, Vec2d.EPSILON);
     }

    /* Must override equals; two Vec2d's are equals iff their coordinates are
     * ==.  The equals method must test first that the supplied object is a Vec2d
     * by using instanceOf (if (!o instanceOf Vec2d)) return false); then cast
     * to a Vec2d so can compare instance vars.
     * 
     */
     
    @Override
    public boolean equals(Object o) 
    {        
        if (!(o instanceof Vec2d)) {
            return false;
        }
        Vec2d v = (Vec2d)o;
        return (v.m_x == m_x && v.m_y == m_y);  
    }

    @Override
    public int hashCode()
    {
        Double sum = m_x + m_y;
        return sum.hashCode();
    }

    @Override
    public String toString()
    {
        return("(" + m_x + "," + m_y + ")");
    }

    public String toPolarString() {
        return("[r: " + getR() + ", theta: " + getTheta() + "]");
    }
}
