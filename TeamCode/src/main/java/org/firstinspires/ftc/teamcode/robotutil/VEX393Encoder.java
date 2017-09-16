package org.firstinspires.ftc.teamcode.robotutil;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

/**
 * Created by Team 7591 on 1/22/2017.
 * VEX EDR 393 Motor Encoder class. Documentation from http://www.vexrobotics.com/encoder-modules.html.
 */

public class VEX393Encoder {

    private I2cDevice device;
    private I2cAddr address;
    private I2cDeviceSynchImpl synchDevice;
    private byte[] signedVelocityCache;
    private byte[] unsignedVelocityCache;
    private byte[] rotationBitsCache;
    private byte[] rotationUpperBitsCache;

    public final class VEX393EncoderAddresses {

        /* The following I2C addresses come from the VEX Robotics website.
         * They can be found here: http://www.vexrobotics.com/encoder-modules.html
         */

        public final static int I2CADDR_DEFAULT = 0x60;     //8 bit address!
        public final static int REG_SIGNED_VELOCITY = 0x3E; //MSB
        public final static int REG_SIGNED_VELOCITY_LENGTH = 2;
        public final static int REG_UNSIGNED_VELOCITY = 0x44; //MSB
        public final static int REG_UNSIGNED_VELOCITY_LENGTH = 2;
        public final static int REG_READ_ROTATION_BITS = 0x40;
        /*
         * The rotation value is a 48-bit number. The
         * Excerpted from the website:
         * Byte0 Rotation tics bits 15-08 (Msb)
         * Byte1 Rotation tics bits 07-00 (Lsb)
         * Byte2 Rotation tics bits 31-24 (Msb)
         * Byte3 Rotation tics bits 23-16 (Lsb)
         * These form the last 32 bits of the rotation tic values
         */
        public final static int REG_READ_ROTATION_LENGTH = 4;
        public final static int REG_READ_UPPER_ROTATION_BITS = 0x46;
        /*
         * Excerpted from the website:
         * Byte0 Rotation tics bits 47-40 (Msb)
         * Byte1 Rotation tics bits 39-32 (Lsb)
         */
        public final static int REG_READ_UPPER_ROTATION_LENGTH = 2;
        public final static int WRITE_CLEAR_DEVICE_COUNTERS = 0x4A;
    }

    public VEX393Encoder(HardwareMap hardwareMap, String name) {
        device = hardwareMap.i2cDevice.get(name);
        // The docs specify that the address must be even, implying that the default address 0x60 is 8-bit.
        // See http://www.robot-electronics.co.uk/i2c-tutorial for more information.
        // TODO: 1/23/17 look at website
        address = I2cAddr.create8bit(VEX393EncoderAddresses.I2CADDR_DEFAULT);
        synchDevice = new I2cDeviceSynchImpl(device, address, false);
        synchDevice.engage();
    }

    /**
     *
     * @return The current signed (positive or negative) velocity of the motor.
     */
    public int getSignedVelocity() {
        signedVelocityCache = synchDevice.read(VEX393EncoderAddresses.REG_SIGNED_VELOCITY, VEX393EncoderAddresses.REG_SIGNED_VELOCITY_LENGTH);
        return ((signedVelocityCache[0] << 8) | (signedVelocityCache[1] & 0xFF));
    }

    /**
     *
     * @return The current speed of the motor, unsigned.
     */
    public int getUnsignedVelocity() {
        unsignedVelocityCache = synchDevice.read(VEX393EncoderAddresses.REG_UNSIGNED_VELOCITY, VEX393EncoderAddresses.REG_UNSIGNED_VELOCITY_LENGTH);
        return (((unsignedVelocityCache[0] & 0xFF) << 8) | (unsignedVelocityCache[1] & 0xFF));
    }

    /**
     * Get the motor's current rotation value. The following are the ratios of ticks per revolution for various gearings:
     * Measures 627.2 ticks per revolution of the output shaft in High Torque Configuration
     * Measures 392 ticks per revolution of the output shaft in High Speed Configuration
     * Measures 261.333 ticks per revolution of the output shaft in Turbo Gear Configuration
     * @return the current number of encoder ticks, a long 48 bit integer.
     */
    public long readRotation() {
        rotationBitsCache = synchDevice.read(VEX393EncoderAddresses.REG_READ_ROTATION_BITS, VEX393EncoderAddresses.REG_READ_ROTATION_LENGTH);
        rotationUpperBitsCache = synchDevice.read(VEX393EncoderAddresses.REG_READ_UPPER_ROTATION_BITS, VEX393EncoderAddresses.REG_READ_UPPER_ROTATION_LENGTH);
        return ((((long)rotationUpperBitsCache[0]) << 40) | (((long)rotationUpperBitsCache[1] & 0xFF) << 32) | (((long)rotationBitsCache[2] & 0xFF) << 24) | ((rotationBitsCache[3] & 0xFF) << 16) | ((rotationBitsCache[0] & 0xFF) << 8) | (rotationBitsCache[1] & 0xFF));
    }

    /**
     * Reset the motor encoder to 0.
     */
    public void clearCounter() {
        synchDevice.write8(VEX393EncoderAddresses.WRITE_CLEAR_DEVICE_COUNTERS, 1); //Not sure if this is supposed to be 1
    }


}
