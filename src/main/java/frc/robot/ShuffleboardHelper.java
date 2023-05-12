package frc.robot;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class ShuffleboardHelper {
    private static boolean debugEnabled = true;

    public static ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

    public static ArrayList<ShuffleboardObject> shuffleboardObjects = new ArrayList<ShuffleboardObject>();

    public static abstract class ShuffleboardObject {
        public final SimpleWidget widget;
        public final GenericEntry entry;
        public boolean isDebug = true;

        public ShuffleboardObject(SimpleWidget widget) {
            this.widget = widget;
            this.entry = widget.getEntry();
        }

        abstract public void update();
    }

    public static class ShuffleboardOutput extends ShuffleboardObject {
        Supplier<Object> supplier;

        public ShuffleboardOutput(SimpleWidget widget, Supplier<Object> supplier) {
            super(widget);

            this.supplier = supplier;
        }

        @Override
        public void update() {
            entry.setValue(supplier.get());
        }

        public ShuffleboardOutput asNotDebug() {
            isDebug = false;
            return this;
        }

        public ShuffleboardOutput withPosition(int x, int y) {
            widget.withPosition(x, y);
            return this;
        }

        public ShuffleboardObject withSize(int width, int height) {
            widget.withSize(width, height);
            return this;
        }

        public ShuffleboardOutput asDial(float min, float max) {
            widget.withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", min, "max", max));
            return this;
        }

        // public ShuffleboardOutput asNumberSlider(float min, float max) {
        //     widget.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", min, "max", max));
        //     return this;
        // }

        public ShuffleboardOutput asNumberBar(float min, float max) {
            widget.withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", min, "max", max));
            return this;
        }
    }

    public static class ShuffleboardInput extends ShuffleboardObject {
        Consumer<Double> consumer;

        public ShuffleboardInput(SimpleWidget widget, Consumer<Double> consumer) {
            super(widget);
            this.consumer = consumer;
        }

        @Override
        public void update() {
            consumer.accept(entry.getDouble(0));
        }

        public ShuffleboardInput asNotDebug() {
            isDebug = false;
            return this;
        }

        public ShuffleboardInput withPos(int x, int y) {
            widget.withPosition(x, y);
            return this;
        }

        public ShuffleboardInput withSize(int width, int height) {
            widget.withSize(width, height);
            return this;
        }

        public ShuffleboardInput asNumberSlider(float min, float max) {
            widget.withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", min, "max", max));
            return this;
        }
    }

    public static ShuffleboardOutput addOutput(String name, ShuffleboardContainer tab, Supplier<Object> supplier) {
        ShuffleboardOutput shuffleboardOutput = new ShuffleboardOutput(tab.add(name, 0), supplier);
        shuffleboardObjects.add(shuffleboardOutput);
        return shuffleboardOutput;
    }

    public static ShuffleboardInput addInput(String name, ShuffleboardContainer tab, Consumer<Double> consumer) {
        ShuffleboardInput shuffleboardInput = new ShuffleboardInput(tab.add(name, 0), consumer);
        shuffleboardObjects.add(shuffleboardInput);
        return shuffleboardInput;
    }
        
    public static void addMotorInfo(String name, ShuffleboardTab tab, CANSparkMax motor, int x, int y) {
        ShuffleboardLayout layout = tab.getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(x, y);
        
        addOutput("Encoder Position", layout, () -> motor.getEncoder().getPosition()).withPosition(0, 1);
        addOutput("Encoder Velocity", layout, () -> motor.getEncoder().getVelocity()).withPosition(0, 2);
        addOutput("Temperature", layout, () -> motor.getMotorTemperature()).withPosition(0, 3);
        addOutput("Motor Output (Amps)", layout, () -> motor.getOutputCurrent()).withPosition(0, 4);
    }

    public static void update() {
        for (ShuffleboardObject shuffleboardObject : shuffleboardObjects) {
            if (shuffleboardObject.isDebug && !debugEnabled)
                continue;
            shuffleboardObject.update();
        }
    }
}
