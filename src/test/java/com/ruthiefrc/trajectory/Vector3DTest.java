package com.ruthiefrc.trajectory;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class Vector3DTest {
    
    @Test
    void testConstruction() {
        Vector3D v = new Vector3D(1.0, 2.0, 3.0);
        assertEquals(1.0, v.x, 1e-10);
        assertEquals(2.0, v.y, 1e-10);
        assertEquals(3.0, v.z, 1e-10);
    }
    
    @Test
    void testAdd() {
        Vector3D v1 = new Vector3D(1.0, 2.0, 3.0);
        Vector3D v2 = new Vector3D(4.0, 5.0, 6.0);
        Vector3D result = v1.add(v2);
        
        assertEquals(5.0, result.x, 1e-10);
        assertEquals(7.0, result.y, 1e-10);
        assertEquals(9.0, result.z, 1e-10);
    }
    
    @Test
    void testSubtract() {
        Vector3D v1 = new Vector3D(5.0, 7.0, 9.0);
        Vector3D v2 = new Vector3D(1.0, 2.0, 3.0);
        Vector3D result = v1.subtract(v2);
        
        assertEquals(4.0, result.x, 1e-10);
        assertEquals(5.0, result.y, 1e-10);
        assertEquals(6.0, result.z, 1e-10);
    }
    
    @Test
    void testScale() {
        Vector3D v = new Vector3D(1.0, 2.0, 3.0);
        Vector3D result = v.scale(2.0);
        
        assertEquals(2.0, result.x, 1e-10);
        assertEquals(4.0, result.y, 1e-10);
        assertEquals(6.0, result.z, 1e-10);
    }
    
    @Test
    void testMagnitude() {
        Vector3D v = new Vector3D(3.0, 4.0, 0.0);
        assertEquals(5.0, v.magnitude(), 1e-10);
    }
    
    @Test
    void testNormalize() {
        Vector3D v = new Vector3D(3.0, 4.0, 0.0);
        Vector3D normalized = v.normalize();
        
        assertEquals(0.6, normalized.x, 1e-10);
        assertEquals(0.8, normalized.y, 1e-10);
        assertEquals(0.0, normalized.z, 1e-10);
        assertEquals(1.0, normalized.magnitude(), 1e-10);
    }
    
    @Test
    void testDot() {
        Vector3D v1 = new Vector3D(1.0, 2.0, 3.0);
        Vector3D v2 = new Vector3D(4.0, 5.0, 6.0);
        
        assertEquals(32.0, v1.dot(v2), 1e-10);
    }
    
    @Test
    void testCross() {
        Vector3D v1 = new Vector3D(1.0, 0.0, 0.0);
        Vector3D v2 = new Vector3D(0.0, 1.0, 0.0);
        Vector3D result = v1.cross(v2);
        
        assertEquals(0.0, result.x, 1e-10);
        assertEquals(0.0, result.y, 1e-10);
        assertEquals(1.0, result.z, 1e-10);
    }
}
