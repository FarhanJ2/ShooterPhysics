package org.steelhawks.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.steelhawks.Constants;

import java.util.function.DoubleSupplier;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns
 * default if not or value not in dashboard.
 *
 * @author farhanj2
 *
 */
public class TunableNumber implements DoubleSupplier {
    private final String key;
    private double defaultValue;
    private double lastValue = defaultValue;

    /**
     * Create a new TunableNumber with a default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default tunable value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = "TunableNumbers/" + dashboardKey;
        setDefault(defaultValue);
    }

    /**
     * Create a new TunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public TunableNumber(String dashboardKey) {
        this(dashboardKey, 0);
    }

    /**
     * Get the default value for the number that has been set
     *
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     *
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.TUNING_MODE) {
            // This makes sure the data is on NetworkTables but will not change it
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     *
     * @return The current value
     */
    public double get() {
        return Constants.TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }

    /**
     * Callback that checks if the value has been modified.
     * This must be in a periodic function for your tunable numbers to take effect.
     *
     * @return If the value was changed or not
     */
    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        }

        return false;
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}