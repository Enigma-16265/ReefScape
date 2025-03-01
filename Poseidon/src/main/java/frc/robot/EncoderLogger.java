package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.logging.DataNetworkTableLog;

public class EncoderLogger
{
    private static final DataNetworkTableLog tlmLog =
        new DataNetworkTableLog( 
            "AbsoluteEncoders",
            Map.of( "voltage0", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "angle0", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "voltage1", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "angle1", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "voltage2", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "angle2", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "voltage3", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "angle3", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Array to hold our 4 encoder inputs.
    private final AnalogInput[] encoders = new AnalogInput[4];
    
    // Define the mapping parameters (adjust if your sensor has a different range)
    private final double maxVoltage = 5.0;
    private final double fullRotationDegrees = 360.0;

    public EncoderLogger() {
        // Initialize the four encoders on channels 0, 1, 2, 3
        for (int i = 0; i < 4; i++) {
            encoders[i] = new AnalogInput(i);
        }
    }

    /**
     * Reads each encoder's voltage, converts it to an angle, and logs the values.
     */
    public void logEncoderData() {
        for (int i = 0; i < encoders.length; i++) {
            // Read the voltage from the encoder
            double voltage = encoders[i].getVoltage();
            
            // Convert the voltage to an angle.
            // For a sensor that maps 0V to 0° and 5V to 360°:
            double angle = (voltage / maxVoltage) * fullRotationDegrees;
            
            if ( i  == 0 )
            {
                tlmLog.publish( "voltage0", voltage );
                tlmLog.publish( "angle0", angle );
            } else if ( i == 1 )
            {
                tlmLog.publish( "voltage1", voltage );
                tlmLog.publish( "angle1", angle );
            } else if ( i == 2 )
            {
                tlmLog.publish( "voltage2", voltage );
                tlmLog.publish( "angle2", angle );
            } else if ( i == 3 )
            {
                tlmLog.publish( "voltage3", voltage );
                tlmLog.publish( "angle3", angle );
            }

        }
    }
}
