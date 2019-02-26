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

package frc.subsystems;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import frc.util.Vec2d;
import static org.junit.Assert.*;

/**
 * @author lwa
 */
public class NavTest {
    /**
     * Relative error for most reasonable size doubles
     */
    public static final double EPS = 0.00001d;
    
    public NavTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }

    /**
     * Test of yawToFieldAngle method, of class Nav
     */
    @Test
    public void testYawtoFieldAngle() {
        System.out.println("yawToFieldAngle");
        double x = 0.0d;
        double y = 90.0d;
        double expResult = y;
        double result = Nav.yawToFieldAngle(x);
        assertEquals(expResult, result, EPS);
        
        expResult = x;
        result = Nav.yawToFieldAngle(y);
        assertEquals(expResult, result, EPS);

        expResult = 180.0;
        result = Nav.yawToFieldAngle(-y);
        assertEquals(expResult, result, EPS);

        expResult = -135.0;
        result = Nav.yawToFieldAngle(-135.0);
        assertEquals(expResult, result, EPS);

        expResult = -60.0;
        result = Nav.yawToFieldAngle(150.0);
        assertEquals(expResult, result, EPS);
    }

    /**
     * Test of yawToVec method, of class Nav.
     */
    @Test
    public void testmakePolar() {
        System.out.println("yawToVec");
        double yaw = 0.0d;
        Vec2d expResult = Vec2d.makeCart(0.0d, 1.0d);
        Vec2d result = Nav.yawToVec(yaw);
        assertTrue(expResult.isNear(result));
 
        yaw = 90.0d;
        expResult = Vec2d.makeCart(1.0d, 0.0d);
        result = Nav.yawToVec(yaw);
        assertTrue(expResult.isNear(result));

        yaw = -150.0d;
        expResult = Vec2d.makePolar(1.0d, Math.toRadians(240.0d));
        result = Nav.yawToVec(yaw);
        assertTrue(expResult.isNear(result));
    }

}
