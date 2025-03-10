package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static final Timer tempStateTimer = new Timer();
    private static final Timer blinkTimer = new Timer();

    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private static LEDState state = LEDState.STARTUP;
    private static boolean blinkOff = false;

    /* Initialize all colors, this allows easy custom color creation */
    private enum robotColor {
        off(0, 0, 0),
        red(255, 0, 0),
        green(0, 255, 0),
        hightideTeal(44, 162, 165),
        blue(0, 0, 255),
        cheesyBlue(0, 112, 255),
        cyan(0, 255, 255),
        magenta(255, 0, 255),
        yellow(255, 255, 0),
        white(255, 255, 255),
        dim(50, 50, 50),
        gsmst(6, 63, 142),
        disabled(200, 0, 0);

        public final int r, g, b;

        robotColor(int r, int g, int b) {
            // this looks incorrect because our Andymark LEDs are GRB, not RGB
            this.r = g;
            this.g = r;
            this.b = b;
        }
    }

    private static class LEDConfig {
        LEDPattern animation = null;
        robotColor color = null;
        boolean blink = false;

        LEDConfig(robotColor color) {
            this.color = color;
        }

        LEDConfig(robotColor color, boolean blink) {
            this.color = color;
            this.blink = true;
        }
        LEDConfig(LEDPattern animation) {
            this.animation = animation;
        }
    }

    public static enum LEDState {
        STARTUP(new LEDConfig(Constants.LEDs.ScrollingGradient)),
        DISABLED(new LEDConfig(robotColor.disabled)),
        NORMAL(new LEDConfig(robotColor.gsmst)),
        INTAKE(new LEDConfig(robotColor.green)),
        EXTAKE(new LEDConfig(robotColor.green, true)),
        ERROR(new LEDConfig(robotColor.red, true)),
        WARNING(new LEDConfig(robotColor.yellow, true));

        public final LEDConfig config;

        LEDState(LEDConfig config) {
            this.config = config;
        } 
    }

    public LEDSubsystem()
    {
        m_led = new AddressableLED(9);

        m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.numLEDs);
        m_led.setLength(m_ledBuffer.getLength());

        LEDPattern pattern = Constants.LEDs.ScrollingGradient;
        pattern.applyTo(m_ledBuffer);

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private static void setColor(robotColor color, int startIdx, int count) {
        Color WPIColor = new Color(color.r, color.g, color.b);
        LEDPattern pattern = LEDPattern.solid(WPIColor);
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    private static void setColor(robotColor color, int count) {
        setColor(color, Constants.LEDs.startIdx, count);
    }

    private static void setColor(robotColor color) {
        setColor(color, Constants.LEDs.numLEDs);
    }

    private static void setColor(LEDPattern pattern) {
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public static void triggerError() {
        state = LEDState.ERROR;
        tempStateTimer.restart();
    }

    public static void triggerWarning() {
        state = LEDState.WARNING;
        tempStateTimer.restart();
    }


     
    @Override
    public void periodic() {
        // set Last State
        LEDState lastState = state;
        // Determine the proper LED state
        if (DriverStation.isDisabled()) {
            if (state != LEDState.STARTUP && state != LEDState.DISABLED) {
                state = LEDState.DISABLED;
            }
        } else {
            // Check if the temporary state should be cleared
            if (tempStateTimer.isRunning() && tempStateTimer.hasElapsed(Constants.LEDs.tempStateTime)) {
                tempStateTimer.stop();
            }
            // Compute the proper state if's not temporarily overridden
            if (!tempStateTimer.isRunning()) {
                // TODO: implement logic for if robot intaking
                if (true) {
                    state = LEDState.INTAKE;
                } else {
                    state = LEDState.NORMAL;
                }        
            }
        }

        // Change the LEDs if the state has changed
        if (state != lastState) {
            switch (state)
            {
                case STARTUP:
                    setColor(state.config.animation);
                    break;
                default:
                    setColor(state.config.color);
            }

            if (state.config.blink) {
                blinkOff = false;
                blinkTimer.restart();
            } else {
                blinkTimer.stop();
            }
        }

        if (blinkTimer.isRunning() && blinkTimer.advanceIfElapsed((state == LEDState.ERROR || state == LEDState.WARNING) ? Constants.LEDs.errorBlinkRate : Constants.LEDs.blinkRate)) {
            blinkOff = !blinkOff;
            setColor(blinkOff ? robotColor.off : state.config.color);
        }
    }
}