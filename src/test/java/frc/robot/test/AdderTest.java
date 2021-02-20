package frc.robot.test;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import frc.robot.Adder;

public class AdderTest {
    private frc.robot.Adder adder;

    @Before
    public void setUp() {
        adder = new Adder();
    }

    @Test
    public void testAdding() {
        Assert.assertEquals(adder.add(2,3), 5);
        Assert.assertEquals(adder.add(3,3), 6);
    }
}
