const string PistonName = "Rotor"; // Name of the piston to control
const double TimeStep = 1.0 / 6.0; // Update10 is 1/6th a second
PID _pid;
IMyMotorStator _piston;
double _desiredExtension = 0;
Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    // This is the simplest PID controller, you can change the gains if you'd like
    _pid = new PID(10, 0, 0, TimeStep);
    
    // Grab our piston
    _piston = GridTerminalSystem.GetBlockWithName(PistonName) as IMyMotorStator;
}

void Main(string arg, UpdateType updateSource)
{
    if (_piston == null)
    {
        Echo($"ERROR: No piston named '{PistonName}'!");
        return;
    }
    
    if (!string.IsNullOrEmpty(arg))
    {
        double val;
        if (double.TryParse(arg, out val))
        {
            // Set desired extension
            _desiredExtension = val/360*2*Math.PI;
        }
    }
    
    if ((updateSource & UpdateType.Update10) != 0)
    {
        // Compute our error
        double error = _desiredExtension - (_piston.Angle);
        if (error>Math.PI)
        {
            error = -1*(Math.PI*2-error);
        }
        if (error<Math.PI*-1)
        {
            error = (Math.PI*2+error);
        }
        
        // Set piston velocity to the result of our PID output
        _piston.TargetVelocityRPM = (float)_pid.Control(error);
    }
    
    Echo($"Desired extension: {_desiredExtension}\nCurrent extension: {_piston.Angle:n2}");
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