/*
    Authors: Ben Ryan
             Yit Chee Chin
             
    This programs will control the EV3 robot seen in the README file.
    It will avoid obstacles and keep itself from falling off the edge
    of things.
*/

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.motor.*;

import java.io.File;
import java.lang.Math;

public class Explorer 
{
    public static void main(String[] args) 
    {
        int angle;
        int maxAngle = 48; // for sweeping ultrasonic sensor
        float forwardDist = 0.3f; // moving forward until this dist
        float reverseDist = 0.15f; // backup if dist reaches this dist
        int reverseAngle = 40; // angle to reverse at
        int angleSpeed = 4; // degrees to sweep sensor per loop
        float circumference = (float) (2 * Math.PI * 2.75); //wheel circumference
        
        //general setup of inputs/outputs
        EV3TouchSensor touch = new EV3TouchSensor(SensorPort.S1);
        SampleProvider touchSP = touch.getTouchMode();
        EV3UltrasonicSensor ultrasonic = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distSP = ultrasonic.getDistanceMode();
        
        EV3ColorSensor sensor3 = new EV3ColorSensor(SensorPort.S3);
        sensor3.setFloodlight(true);
        int colorSP = sensor3.getColorID();
        EV3ColorSensor sensor4 = new EV3ColorSensor(SensorPort.S4);
        sensor4.setFloodlight(true);
        int colorSP2 = sensor4.getColorID();
        Sound.setVolume(30);
        
        float [] sample = new float[distSP.sampleSize()];

        System.out.println("Explore");
        System.out.println("Press any key to start");
        
        Button.LEDPattern(4); // flash green led and
        Sound.beepSequenceUp(); // make sound when ready.
        Button.waitForAnyPress();
        
        // create motor objects to control the motors.
        EV3LargeRegulatedMotor right = new EV3LargeRegulatedMotor(MotorPort.A);
        EV3LargeRegulatedMotor left = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3MediumRegulatedMotor sweep = new EV3MediumRegulatedMotor(MotorPort.C);
        
        double pi = Math.PI;
        int gLength = (int) (12 * reverseAngle * (pi/180));
        int gArc = (int)(gLength  / circumference *360);
        
        // loop until touch sensor is touched
        while(!isTouched(touchSP))
        {
            //get inputs for this loop
            distSP.fetchSample(sample, 0);
            float dist = sample[0];
            colorSP = sensor3.getColorID();
            colorSP2 = sensor4.getColorID();
            
            //if no color on either sensor, sensor has gone over edge
            //stop, backup, turn away from edge
            if (colorSP == Color.NONE || colorSP2 == Color.NONE)
            {
                System.out.println("No color");
                left.setSpeed(0);
                right.setSpeed(0);
                Delay.msDelay(500);
                
                left.setSpeed(180);
                right.setSpeed(180);
                left.backward();
                right.backward();
                Delay.msDelay(2000);
                
                left.setSpeed(0);
                right.setSpeed(0);
                Delay.msDelay(500);
                
                right.setSpeed(gArc);
                right.forward();
                left.setSpeed(gArc);
                left.backward();
                Delay.msDelay(1000);  
            }
            //forward, normal operation
            else if(dist > forwardDist) 
            {
                left.forward();
                right.forward();
                left.setSpeed(270);
                right.setSpeed(270);
            }
            //turning away from obstacle
            else if(dist <= forwardDist && dist >= reverseDist)
            {
                angle = 0;
                boolean backward = false;

                left.setSpeed(0);
                right.setSpeed(0);
                
                //look for direction without obstacle by sweeping 
                //the ultrasonic sensor mounted on the motor
                while(dist < 0.4 && !isTouched(touchSP))
                {
                    distSP.fetchSample(sample, 0);
                    dist = sample[0];
                    
                    //sweep one way
                    if(backward == true)
                    {
                        sweep.rotate(-angleSpeed);
                        angle-=angleSpeed;
                    }
                    else//sweep other way
                    {
                        sweep.rotate(angleSpeed);
                        angle+=angleSpeed;
                    }
                    
                    // when max reached in one direction swap
                    if(angle >= maxAngle)
                    {
                        backward = true;
                    }
                    //no path found turn 90 and search again
                    else if(angle <= -1 * maxAngle)
                    {
                        System.out.println("No path found");
                        
                        right.setSpeed(gArc);
                        right.forward();
                        left.setSpeed(gArc);
                        left.backward();
                        Delay.msDelay(1000);
                        backward = false;
                        right.setSpeed(0);
                        left.setSpeed(0);
                        
                        sweep.rotate(angle * -1);
                        angle = 0;
                    }
                }
                
                //calclate required speed for desired turn
                int length = (int) (12 * angle * (pi/180));
                int arc = (int)(length  / circumference *360);
        
                //determine which wheel should turn
                if (angle > 0)
                {
                    right.setSpeed(arc);
                    right.forward();
                }
                else
                {
                    left.setSpeed(arc);
                    left.forward();
                }
                Delay.msDelay(1000);
                            
                //reset ultrasonic sensor orientation
                sweep.rotate(angle * -1);
                
            }
            //reverse if too close to obstacle
            else if(dist <= reverseDist)
            {
                left.setSpeed(180);
                right.setSpeed(180);
                left.backward();
                right.backward();
                Sound.buzz();
                Delay.msDelay(1500);
            }
        }//end while()
        
        // stop motors with brakes on.
        left.stop();
        right.stop();
        
        // free up resources.
        left.close();
        right.close();
        sweep.close();
        
        sensor3.close();
        sensor4.close();
        touch.close();
        ultrasonic.close();
        Sound.beepSequence(); // we are done.
    }
    
    // method to read touch sensor and return true or false if touched.
    private static boolean isTouched(SampleProvider sp)
    {
        float [] sample = new float[sp.sampleSize()];
        sp.fetchSample(sample, 0);
        if (sample[0] == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    
    private static int getArcLength(int alpha)
    {
        double pi = Math.PI;
        float radius = 12;
        
        int length = (int) (radius * alpha    * (pi/180));
        return length;
        
    }
}
