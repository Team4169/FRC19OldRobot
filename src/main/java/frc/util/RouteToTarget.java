/*
 * The MIT License
 *
 * Copyright 2019 lwa.
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
 * Utility class to hold the vectors the describe the route the robot needs
 * to follow to get from its current location to a vision target.  The 
 * intention is that, when the robot arrives at the target, it will be aligned
 * properly (normal i.e. perpendicular to the target).
 */
public class RouteToTarget {

    /**
     * The vector directly from the camera lens to the target.
     */
    private final Vec2d m_targetDirectVec;

    /**
     * The vector from the robot, in the robot's current direction of travel,
     * to the target normal vector.
     */
    private final Vec2d m_interceptVec;

    /**
     * The vector from the intercept vector to the target, normal (i.e. perpendicular)
     * to the plane in which the target lies.  The invariant is that :
     *    m_normalVec + m_interceptVec = m_targetDirectVec
     */
    private final Vec2d m_normalVec;

    /**
     * Constructor given the three vectors
     * @param targetDirectVec Vector directly from cameral lens to target
     * @param interceptVec Vector from robot to normal vec
     * @param normalVec Vector from normal vec to target
     */
    public RouteToTarget(Vec2d targetDirectVec, Vec2d interceptVec, Vec2d normalVec) {

        m_targetDirectVec = targetDirectVec;
        m_interceptVec = interceptVec;
        m_normalVec = normalVec;
    }

    /**
     * Return the target direct vector
     * @return target direct vector
     */
    public Vec2d getTargetDirectVec() {
        return m_targetDirectVec;
    }

    /**
     * Return the intercept vector
     * @return intercept vector
     */
    public Vec2d getInterceptVec() {
        return m_interceptVec;
    }

       /**
     * Return the normal vector
     * @return normal vector
     */
    public Vec2d getNormalVec() {
        return m_normalVec;
    }
}
