package com.ruthiefrc.trajectory.empirical;

import com.ruthiefrc.trajectory.Vector3D;
import org.junit.jupiter.api.*;
import java.io.File;
import java.io.IOException;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for ShotOutcomeModel.
 */
public class ShotOutcomeModelTest {
    
    private ShotOutcomeModel model;
    
    @BeforeEach
    public void setUp() {
        model = new ShotOutcomeModel();
    }
    
    @Test
    public void testRecordShot() {
        model.recordShot(
            1.0, 2.0, 0.5,  // robot pose
            0.1, 0.3,       // turret/hood
            12.0, 200.0, new Vector3D(0, 1, 0),  // velocity/spin
            3.5,  // distance
            ShotOutcome.MADE_SHOT, RimLocation.NONE,
            0.0, 0.0  // errors
        );
        
        assertEquals(1, model.getShotCount());
    }
    
    @Test
    public void testSuccessRate() {
        // Record 3 successful shots
        for (int i = 0; i < 3; i++) {
            model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
                new Vector3D(0, 1, 0), 3.5, ShotOutcome.MADE_SHOT, 
                RimLocation.NONE, 0.0, 0.0);
        }
        
        // Record 2 failed shots
        for (int i = 0; i < 2; i++) {
            model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
                new Vector3D(0, 1, 0), 3.5, ShotOutcome.MISSED, 
                RimLocation.NONE, 0.0, 0.0);
        }
        
        assertEquals(0.6, model.getSuccessRate(), 0.01);
    }
    
    @Test
    public void testRimInCountsAsSuccess() {
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.RIM_IN, 
            RimLocation.FRONT, 0.0, 0.0);
        
        assertEquals(1.0, model.getSuccessRate(), 0.01);
    }
    
    @Test
    public void testGetShotsInRange() {
        // Add shots at different distances
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 2.0, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 5.0, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 8.0, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        
        List<ShotRecord> midRange = model.getShotsInRange(3.0, 6.0);
        assertEquals(1, midRange.size());
        assertEquals(5.0, midRange.get(0).distanceToTarget, 0.01);
    }
    
    @Test
    public void testRimOutRate() {
        // Add rim-outs at different locations
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.RIM_OUT, 
            RimLocation.FRONT, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.RIM_OUT, 
            RimLocation.FRONT, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.RIM_OUT, 
            RimLocation.BACK, 0.0, 0.0);
        
        assertEquals(2.0/3.0, model.getRimOutRate(RimLocation.FRONT), 0.01);
        assertEquals(1.0/3.0, model.getRimOutRate(RimLocation.BACK), 0.01);
    }
    
    @Test
    public void testCSVExportImport() throws IOException {
        // Add some test data
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.05, -0.02);
        model.recordShot(2.0, 3.0, 1.0, 0.2, 0.4, 13.0, 250.0, 
            new Vector3D(0.1, 0.9, 0), 4.5, ShotOutcome.RIM_OUT, 
            RimLocation.FRONT, -0.03, 0.01);
        
        // Export
        String tempFile = "/tmp/test_shots.csv";
        model.exportToCSV(tempFile);
        
        // Import into new model
        ShotOutcomeModel model2 = new ShotOutcomeModel();
        model2.importFromCSV(tempFile);
        
        // Verify
        assertEquals(2, model2.getShotCount());
        List<ShotRecord> shots = model2.getAllShots();
        assertEquals(3.5, shots.get(0).distanceToTarget, 0.01);
        assertEquals(ShotOutcome.MADE_SHOT, shots.get(0).outcome);
        
        // Cleanup
        new File(tempFile).delete();
    }
    
    @Test
    public void testJSONExportImport() throws IOException {
        // Add some test data
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.05, -0.02);
        
        // Export
        String tempFile = "/tmp/test_shots.json";
        model.exportToJSON(tempFile);
        
        // Import into new model
        ShotOutcomeModel model2 = new ShotOutcomeModel();
        model2.importFromJSON(tempFile);
        
        // Verify
        assertEquals(1, model2.getShotCount());
        List<ShotRecord> shots = model2.getAllShots();
        assertEquals(3.5, shots.get(0).distanceToTarget, 0.01);
        assertEquals(ShotOutcome.MADE_SHOT, shots.get(0).outcome);
        
        // Cleanup
        new File(tempFile).delete();
    }
    
    @Test
    public void testClear() {
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 3.5, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        assertEquals(1, model.getShotCount());
        
        model.clear();
        assertEquals(0, model.getShotCount());
    }
    
    @Test
    public void testSuccessRateInRange() {
        // Add shots at different distances with different outcomes
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 2.0, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 5.0, ShotOutcome.MADE_SHOT, 
            RimLocation.NONE, 0.0, 0.0);
        model.recordShot(1.0, 2.0, 0.5, 0.1, 0.3, 12.0, 200.0, 
            new Vector3D(0, 1, 0), 5.5, ShotOutcome.MISSED, 
            RimLocation.NONE, 0.0, 0.0);
        
        double midRangeSuccess = model.getSuccessRateInRange(4.0, 6.0);
        assertEquals(0.5, midRangeSuccess, 0.01);  // 1 of 2 shots succeeded
    }
}
