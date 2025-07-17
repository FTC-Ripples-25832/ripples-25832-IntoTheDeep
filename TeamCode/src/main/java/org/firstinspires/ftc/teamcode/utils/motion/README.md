this broken ai

# Motion Profiling Implementation

This directory contains the motion profiling system for smooth slide movements in the FTC robot.

## Files

### TrapezoidalMotionProfile.java
- Core trapezoidal motion profile implementation
- Generates smooth acceleration, constant velocity, and deceleration phases
- Handles both trapezoidal and triangular profiles based on distance

### MotionProfileController.java
- Integrates motion profiles with existing PIDF controllers
- Provides position, velocity, and acceleration feedforward
- Manages profile execution and completion detection

## Usage in Subsystems

### LowerSlide
```java
// Enable motion profiling and move to 25cm
lowerSlide.setPositionCMWithProfile(25.0);

// Check if motion profile is active
if (lowerSlide.isMotionProfileActive()) {
    double timeRemaining = lowerSlide.getMotionProfileTimeRemaining();
}
```

### UpperSlide
```java
// Enable motion profiling and move to 50cm
upperSlide.setPositionCMWithProfile(50.0);

// Check if motion profile is active
if (upperSlide.isMotionProfileActive()) {
    double timeRemaining = upperSlide.getMotionProfileTimeRemaining();
}
```

## Command Usage

### MotionProfiledSlideCommand
```java
// Move lower slide only
MotionProfiledSlideCommand.lowerSlideOnly(lowerSlide, 30.0);

// Move upper slide only
MotionProfiledSlideCommand.upperSlideOnly(upperSlide, 75.0);

// Move both slides simultaneously
new MotionProfiledSlideCommand(lowerSlide, upperSlide, 25.0, 50.0);
```

## Configuration

Motion profile parameters are configurable via FTC Dashboard in ConfigVariables:

### LowerSlideVars
- `MAX_VELOCITY_CM_S`: Maximum velocity (default: 30.0 cm/s)
- `MAX_ACCELERATION_CM_S2`: Maximum acceleration (default: 50.0 cm/s²)
- `VELOCITY_FEEDFORWARD`: Velocity feedforward gain (default: 0.0)
- `ACCELERATION_FEEDFORWARD`: Acceleration feedforward gain (default: 0.0)

### UpperSlideVars
- `MAX_VELOCITY_CM_S`: Maximum velocity (default: 25.0 cm/s)
- `MAX_ACCELERATION_CM_S2`: Maximum acceleration (default: 40.0 cm/s²)
- `VELOCITY_FEEDFORWARD`: Velocity feedforward gain (default: 0.0)
- `ACCELERATION_FEEDFORWARD`: Acceleration feedforward gain (default: 0.0)

## Benefits

1. **Smoother Movement**: Controlled acceleration/deceleration prevents slip
2. **Better Precision**: Reduces overshoot and oscillation
3. **Reduced Wear**: Less stress on mechanical components
4. **Consistent Timing**: Predictable movement duration
5. **Improved Reliability**: Works regardless of battery voltage
