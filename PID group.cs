const double TimeStep = 1.0 / 6.0; // Update10 is 1/6th a second
PID _pid;
List<IMyMotorStator> _rotors = new List<IMyMotorStator>();
Dictionary<IMyMotorStator, bool> _invertMap = new Dictionary<IMyMotorStator, bool>();
double _desiredAngle = 0; // Declare _desiredAngle at the class level

Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    // This is the simplest PID controller, you can change the gains if you'd like
    _pid = new PID(1, 0, 0, TimeStep);
    
    // Grab our rotor group
    IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName("Rotors PID");
    if (group == null)
    {
        Echo("Group not found");
        return;
    }
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    group.GetBlocks(blocks);
    foreach (var block in blocks)
    {
        if (block is IMyMotorStator)
        {
            _rotors.Add(block as IMyMotorStator);
            _invertMap.Add(block as IMyMotorStator, (block as IMyMotorStator).CustomName.ToLower().Contains("inv"));
        }
    }
}void Main(string arg, UpdateType updateSource)
{
    Echo("Debug Info:");

    if (_rotors.Count == 0)
    {
        Echo("ERROR: No rotors found in group");
        return;
    }
    
    if (!string.IsNullOrEmpty(arg))
    {
        double val;
        if (double.TryParse(arg, out val))
        {
            _desiredAngle = val; // Set _desiredAngle here
            
            foreach (var rotor in _rotors)
            {
                double desiredAngle = val;
                
                // Adjust desired angle based on rotor inversion
                if (_invertMap[rotor])
                {
                    desiredAngle = 360 - val;
                }
                
                // Compute error
                double currentAngle = rotor.Angle * 180 / Math.PI;
                double error = (desiredAngle - currentAngle) % 360;
                if (error > 180)
                {
                    error -= 360;
                }
                else if (error < -180)
                {
                    error += 360;
                }
                
                rotor.TargetVelocityRPM = (float)_pid.Control(error);

                // Debugging info
                Echo($"Rotor '{rotor.CustomName}': Current Angle: {currentAngle}, Desired Angle: {desiredAngle}, Error: {error}");
            }
        }
    }
    
    if ((updateSource & UpdateType.Update10) != 0)
    {
        foreach (var rotor in _rotors)
        {
            double currentAngle = rotor.Angle * 180 / Math.PI;
            double error = _invertMap[rotor] ? 360 - _desiredAngle : _desiredAngle;
            error -= currentAngle;
            if (error > 180)
            {
                error -= 360;
            }
            if (error < -180)
            {
                error += 360;
            }

            rotor.TargetVelocityRPM = (float)_pid.Control(error);

            // Debugging info
            Echo($"Rotor '{rotor.CustomName}': Current Angle: {currentAngle}, Desired Angle: {_desiredAngle}, Error: {error}");
        }
    }
}



public class PID
{
    public double Kp { get; set; } = 0;
    public double Ki { get; set; } = 0;
    public double Kd { get; set; } = 0;
    public double Value { get; private set; }

    double _timeStep = 0;
    double _inverseTimeStep = 0;
    double _errorSum = 0;
    double _lastError = 0;
    bool _firstRun = true;

    public PID(double kp, double ki, double kd, double timeStep)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        _timeStep = timeStep;
        _inverseTimeStep = 1 / _timeStep;
    }

    protected virtual double GetIntegral(double currentError, double errorSum, double timeStep)
    {
        return errorSum + currentError * timeStep;
    }

    public double Control(double error)
    {
        //Compute derivative term
        double errorDerivative = (error - _lastError) * _inverseTimeStep;

        if (_firstRun)
        {
            errorDerivative = 0;
            _firstRun = false;
        }

        //Get error sum
        _errorSum = GetIntegral(error, _errorSum, _timeStep);

        //Store this error as last error
        _lastError = error;

        //Construct output
        Value = Kp * error + Ki * _errorSum + Kd * errorDerivative;
        return Value;
    }

    public double Control(double error, double timeStep)
    {
        if (timeStep != _timeStep)
        {
            _timeStep = timeStep;
            _inverseTimeStep = 1 / _timeStep;
        }
        return Control(error);
    }

    public void Reset()
    {
        _errorSum = 0;
        _lastError = 0;
        _firstRun = true;
    }
}
