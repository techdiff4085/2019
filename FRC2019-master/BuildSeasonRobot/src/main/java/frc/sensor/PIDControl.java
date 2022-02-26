/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.sensor;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class PIDControl extends SendableBase {

    // Tuning Variables
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    // Storing all error for use and reporting
    private double error;
    //Used to avoid situations were loop runs too fast and derivative returns as 0
    private double maxPreviousErrors = 5;
    private ArrayList<Double> previousErrors = new ArrayList<>();
    private double previousError;
    private double totalError;
    public boolean acumulateError = true;

    // The max abs(value) the total error can reach
    private double maxTotalError;

    //The integral error when only start ramping when the derivative error is less the this rate
    private double integralKickInRate;

    // Time in between calculate calls
    private double deltaTime = 0.02;

    // Operation variables
    private double minOutput;
    private double maxOutput;

    private double minInput;
    private double maxInput;
    private double inputRange;
    private boolean continuous;

    private double setpoint;

    private double tolerance;

    public PIDControl(double p) {
        this(p, 0, 0, 0);
    }

    public PIDControl(double p, double i) {
        this(p, i, 0, 0);
    }

    public PIDControl(double p, double i, double d) {
        this(p, i, d, 0);
    }

    public PIDControl(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;

        reset();
    }

    public void reset() {
        this.totalError = 0;
        this.previousError = 0;
        this.error = 0;
        previousErrors.clear();
    }

    public void setPID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public void setP(double p) {
        kP = p;
    }

    public void setI(double i) {
        
        kI = i;
    }

    public void setD(double d) {
        kD = d;
    }

    public void setF(double f) {
        kF = f;
    }

    /*
     * @return proportional coefficent
     */
    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }
    
    // Set the set point using the logic of min and max input
    public void setSetpoint(double setpoint) {
        if (maxInput > minInput) {
            if (setpoint > maxInput) {
                this.setpoint = maxInput;
            } else if (setpoint < minInput) {
                this.setpoint = minInput;
            } else {
                this.setpoint = setpoint;
            }
        } else {
            this.setpoint = setpoint;
        }
    }

    public double getSetpoint() {
        return setpoint;
    }

    // How small the error must be to stop changing motor speed;
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double getTolerance() {
        return tolerance;
    }

    // Set the min and max values the PIDController can set to
    public void setInputRange(double min, double max) {
        minInput = min;
        maxInput = max;
        inputRange = max - min;
        // Update the setpoint to be in these values if changed mid run
        setSetpoint(setpoint);
    }

    public void setContinuous() {
        continuous = true;
    }

    // Set the min and max values the PIDController can output
    public void setOutputRange(double min, double max) {
        minOutput = min;
        maxOutput = max;
    }

    // Returns the last error recorded when calculate was last called
    // If called in the robot loop, should be every 0.02 seconds
    public double getError() {
        return error;
    }

    // Sets the max total error in the PIDUnit
    public void setMaxTotalError(double max) {
        maxTotalError = max;
    }

    // Sets the max total error based on the current kI and max contribution
    public void setMaxIContribution(double contribution) {
        maxTotalError = contribution / kI;
    }

    //Set the rate the derivative error must be less than
    //for the integral to start ramping up
    public void setIKickInRate(double maxDeltaError) {
        integralKickInRate = Math.abs(maxDeltaError);
    }

    // Returns the total error recorded by the integral term
    public double getTotalError() {
        return totalError;
    }

    // Uses reading given to determine speed based off error
    public double calculate(double input) {
        return calculate(input, 0);
    }

    // Uses reading given and feedforward term to determine speed based off error
    public double calculate(double input, double feedForward) {
        error = getContinuousError(setpoint - input);

        double deltaError = error - previousError;

        previousErrors.add(deltaError);

        if(previousErrors.size() > maxPreviousErrors) {
            //Double check to make sure this is removing at index and not where [i] = 0
            previousErrors.remove(0);
        }
        
        double totalChange = 0;
        for(double n : previousErrors) {
            totalChange += n;
        }

        double derivativeError = totalChange / (deltaTime * previousErrors.size());
        double integralError;

        //Change is small enough to warrant start using the integral or integralKickInRate is not used
        if(Math.abs(derivativeError) < integralKickInRate || integralKickInRate == 0) {
            //Update the integral/total error
            if(maxTotalError == 0 || totalError < maxTotalError) {
                if(acumulateError && !onTarget()) {
                    totalError += error;
                }
            }
        }

        integralError = totalError;
        
        //Compute the motor speed
        double output = kF * feedForward + kP * error + kI * integralError + kD * derivativeError;
        output = clamp(output, minOutput, maxOutput);
        previousError = error;
        return output;
    }

    //For continous systems
    double getContinuousError(double error) {
        if (continuous && inputRange > 0) {
          error %= inputRange;
          if (Math.abs(error) > inputRange / 2) {
            if (error > 0) {
              return error - inputRange;
            } else {
              return error + inputRange;
            }
          }
        }
    
        return error;
      }

    // Returns the instantaneous moments for which the controller is on target
    public boolean onTarget() {
        return Math.abs(error) < tolerance;
    }

    //Initializes the smartdashboard with input fields for each of the PID values
    //Using the given name
    public void initSmartDashboard(String name) {
        SmartDashboard.putNumber(name + "P", getP());
        SmartDashboard.putNumber(name + "I", getI());
        SmartDashboard.putNumber(name + "D", getD());
        SmartDashboard.putNumber(name + "F", getF());
    }

    //Updates the PID values from the smartdashboard finding fields with the given name
    //This should only be called after initSmartDashboard is called and must be called repeatedly for
    //continous updating
    public void updateFromSmartDashboard(String name) {
        setP(SmartDashboard.getNumber(name + "P", getP()));
        setI(SmartDashboard.getNumber(name + "I", getI()));
        setD(SmartDashboard.getNumber(name + "D", getD()));
        setF(SmartDashboard.getNumber(name + "F", getF()));
    }

    private static double clamp(double n, double min, double max) {
        return Math.max(min, Math.min(n, max));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("PIDControl");
      builder.setSafeState(this::reset);
      builder.addDoubleProperty("p", this::getP, this::setP);
      builder.addDoubleProperty("i", this::getI, this::setI);
      builder.addDoubleProperty("d", this::getD, this::setD);
      builder.addDoubleProperty("f", this::getF, this::setF);
      builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
