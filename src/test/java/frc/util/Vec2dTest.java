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

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 * @author lwa
 */
public class Vec2dTest {
    /**
     * Relative error for most reasonable size doubles
     */
    public static final double EPS = 0.00001d;
    
    public Vec2dTest() {
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
     * Test of makeCart method, of class Vec2d.
     */
    @Test
    public void testMakeCart() {
        System.out.println("makeCart");
        double x = 0.0d;
        double y = 0.0d;
        Vec2d expResult = Vec2d.ZERO;
        Vec2d result = Vec2d.makeCart(x, y);
        assertEquals(expResult, result);
        assertEquals(0.0d, expResult.getX(), EPS);
        assertEquals(0.0d, expResult.getY(), EPS);
        assertEquals(0.0d, expResult.getR(), EPS);
        assertEquals(0.0d, expResult.getTheta(), EPS);
        
        x = 3.0d;
        y = 4.0d;
        result = Vec2d.makeCart(x, y);
        assertEquals(x, result.getX(), EPS);
        assertEquals(y, result.getY(), EPS);
        assertEquals(5.0, result.getR(), EPS);
        assertEquals(Math.asin(4.0/5.0), result.getTheta(), EPS);
        
        // 2nd quadrant
        x = -3.0d;
        y = 4.0d;
         result = Vec2d.makeCart(x, y);
        assertEquals(x, result.getX(), EPS);
        assertEquals(y, result.getY(), EPS);
         assertEquals(5.0, result.getR(), EPS);
        assertEquals(Math.acos(-3.0/5.0), result.getTheta(), EPS);
            
        x = 1.0d;
        y = 1.0d;
        result = Vec2d.makeCart(x, y);
        double expDResult = Math.sqrt(2.0d);
        assertEquals(expDResult, result.getR(), EPS);
        expDResult = Math.PI / 4.0d;
        assertEquals(expDResult, result.getTheta(), EPS);
   }

    /**
     * Test of makePolar method, of class Vec2d.
     */
    @Test
    public void testmakePolar() {
        System.out.println("makePolar");
        double r = 0.0d;
        double theta = 0.0d;
        Vec2d expResult = Vec2d.ZERO;
        Vec2d result = Vec2d.makePolar(r, theta);
        assertEquals(expResult, result);
        
        // 4th quadrant
        r = 1.0;
        theta = -Math.PI / 6.0;
        result = Vec2d.makePolar(r, theta);
        assertEquals(r, result.getR(), EPS);
        assertEquals(theta, result.getTheta(), EPS);
        assertEquals(Math.sqrt(3.0d)/2.0d, result.getX(), EPS);
        assertEquals(-0.5d, result.getY(), EPS);
        
        // Special cases at X == 0
        r = 2.0;
        theta = Math.PI / 2.0;
        result = Vec2d.makePolar(r, theta);
        assertEquals(r, result.getR(), EPS);
        assertEquals(theta, result.getTheta(), EPS);
        assertEquals(0.0d, result.getX(), EPS);
        assertEquals(2.0d, result.getY(), EPS);

        // Special cases at X == 0
        r = 2.0;
        theta = -Math.PI / 2.0;
        result = Vec2d.makePolar(r, theta);
        assertEquals(r, result.getR(), EPS);
        assertEquals(theta, result.getTheta(), EPS);
        assertEquals(0.0d, result.getX(), EPS);
        assertEquals(-2.0d, result.getY(), EPS);

    }

   
    /**
     * Test of add method, of class Vec2d.
     */
    @Test
    public void testAdd() {
        System.out.println("add");
        Vec2d a = Vec2d.makeCart(1.0d, 1.0d);
        Vec2d instance = Vec2d.makeCart(2.0d, 2.0d);
        Vec2d expResult = Vec2d.ZERO;
        Vec2d result = Vec2d.ZERO.add(Vec2d.ZERO);
        assertEquals(expResult, result);
        
        expResult = Vec2d.makeCart(3.0, 3.0);
        result = instance.add(a);
        assertEquals(expResult, result);
        
        a = Vec2d.makeCart(-2.0d, -2.0d);
        expResult = Vec2d.ZERO;
        result = instance.add(a);
        assertEquals(expResult, result);
        
        instance = Vec2d.makePolar(10.0d, Math.PI/4.0);
        a = Vec2d.makePolar(10.0d, 3.0*Math.PI/4.0);
        result = instance.add(a);
        
        double r = result.getR();
        double theta = result.getTheta();
        assertEquals(r, 10.0d*Math.sqrt(2.0d), EPS);
        assertEquals(theta, Math.PI/2.0, EPS);
    }

    /**
     * Test of sub method, of class Vec2d.
     */
    @Test
    public void testSub() {
        System.out.println("sub");
        Vec2d a = Vec2d.makeCart(1.0d, 1.0d);
        Vec2d instance = Vec2d.makeCart(2.0d, 2.0d);
        Vec2d expResult = Vec2d.ZERO;
        Vec2d result = Vec2d.ZERO.sub(Vec2d.ZERO);
        assertEquals(expResult, result);
        
        expResult = a;
        result = instance.sub(a);
        assertEquals(expResult, result);
        
        expResult = Vec2d.ZERO;
        result = instance.sub(instance);
        assertEquals(expResult, result);
        
        a = instance;
        instance = Vec2d.makeCart(3.0d, 3.0d);
        expResult = Vec2d.makeCart(-1.0d, -1.0d);
        result = a.sub(instance);
        assertEquals(expResult, result);
      }

    /**
     * Test of mulScalar method, of class Vec2d.
     */
    @Test
    public void testMulScalar() {
        System.out.println("mulScalar");
        double m = 2.0;
        Vec2d a = Vec2d.makeCart(1.0, 1.0);
        Vec2d expResult = Vec2d.makeCart(2.0, 2.0);
        Vec2d result = a.mulScalar(m);
        assertEquals(expResult, result);
        
        result = a.mulScalar(0.0d);
        assertEquals(Vec2d.ZERO, result);
        
        a = Vec2d.makeCart(-5.0d, 12.0d);
        expResult = Vec2d.makeCart(15.0d, -36.0d);
        result = a.mulScalar(-3.0d);
        assertEquals(expResult, result);
  }

  /**
   * Test of getNormal() method, of class Vec2d
   */
  @Test
  public void testGetNormal() {
    System.out.println("getNormal");
    Vec2d i = Vec2d.makeCart(1.0d, 0.0d);
    Vec2d j = Vec2d.makeCart(0.0d, 1.0d);
    Vec2d result = j.getNormal();
    Vec2d expResult = i;
    assertEquals(expResult, result);

    // Verify dot product is zero
    double dotProd = j.dotProduct(result);
    assertEquals(0.0d, dotProd, EPS);

    result = i.getNormal();
    expResult = j.negate();
    assertEquals(expResult, result);

    dotProd = i.dotProduct(result);
    assertEquals(0.0d, dotProd, EPS);

    Vec2d fivePiOverFour = Vec2d.makePolar(1.0d, 5.0d*Math.PI/4.0d);
    result = fivePiOverFour.getNormal();
    expResult = Vec2d.makePolar(1.0d, 3.0d*Math.PI/4.0d);
    dotProd = fivePiOverFour.dotProduct(result);
    assertEquals(0.0d, dotProd, EPS);
  }

    /**
     * Test of hashCode method, of class Vec2d.
     */
    @Test
    public void testHashCode() {
        System.out.println("hashCode");
        Vec2d instance = Vec2d.ZERO;
        Vec2d a = Vec2d.makeCart(0.0d, 0.0d);
        int expResult = instance.hashCode();
        int result = a.hashCode();
        assertEquals(expResult, result);
        
        instance = Vec2d.makeCart(-2.0d, -2.0d);
        a = Vec2d.makeCart(instance.getX(), instance.getY());
        assertEquals(instance, a);
        expResult = instance.hashCode();
        result = a.hashCode();
        assertEquals(expResult, result);
        
        instance = Vec2d.makePolar(2.0d*Math.sqrt(2.0), 5.0d*Math.PI/4.0d);
        a = Vec2d.makeCart(instance.getX(), instance.getY());
        assertEquals(instance, a);
        expResult = instance.hashCode();
        result = a.hashCode();
        assertEquals(expResult, result);
  
     }

    /**
     * Test of toString method, of class Vec2d.
     */
    @Test
    public void testToString() {
        System.out.println("toString");
        Vec2d instance = Vec2d.ZERO;
        String result = instance.toString();
        System.out.println(result);
        assertNotNull(result);
        instance = Vec2d.makePolar(2.0d, Math.PI / 2.0d);
        result = instance.toPolarString();
        System.out.println(result);
        assertNotNull(result);
    }

}
