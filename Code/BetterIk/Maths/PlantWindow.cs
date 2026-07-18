namespace BetterIk.Maths;

/// <summary>
/// Trailing-minimum tracker over a fixed time window. Feed it one plant-point height per
/// frame with the current time; <see cref="Push"/> returns the lowest value seen within the
/// last <see cref="WindowSeconds"/>. Plant-to-ground foot placement uses it to find the
/// "planted" level of a cyclic animation (the lowest the foot reached recently), so that only
/// that constant is removed and authored lifts above it are preserved.
///
/// This is the one stateful helper in the Maths layer; the solvers themselves stay pure. It is
/// engine-agnostic (floats and a time value only), so it is unit-testable without an editor,
/// like the rest of the core. Samples are assumed pushed in non-decreasing time order (the
/// engine wall-clock during a frame loop); the newest <see cref="Capacity"/> samples are
/// retained, so at very high frame rates a window longer than the buffer can hold is measured
/// over as many recent samples as fit rather than the full duration.
/// </summary>
public sealed class PlantWindow
{
    private readonly float[] _time;
    private readonly float[] _value;
    private int _head;   // index of the next write
    private int _count;

    /// <summary>Length of the trailing window in the same time unit passed to <see cref="Push"/>.</summary>
    public float WindowSeconds { get; set; }

    public int Capacity => _time.Length;
    public bool HasSamples => _count > 0;

    public PlantWindow(float windowSeconds, int capacity = 512)
    {
        if (capacity < 1)
            capacity = 1;
        WindowSeconds = windowSeconds;
        _time = new float[capacity];
        _value = new float[capacity];
    }

    /// <summary>Discard all samples (e.g. on a hard clip cut) so the window rebuilds from scratch.</summary>
    public void Reset()
    {
        _head = 0;
        _count = 0;
    }

    /// <summary>
    /// Record a sample and return the trailing minimum over the window ending at
    /// <paramref name="now"/>. With an empty history this is just <paramref name="value"/>.
    /// </summary>
    public float Push(float now, float value)
    {
        _time[_head] = now;
        _value[_head] = value;
        _head = (_head + 1) % _time.Length;
        if (_count < _time.Length)
            _count++;

        return MinSince(now);
    }

    /// <summary>Trailing minimum over the window ending at <paramref name="now"/> without adding a sample.</summary>
    public float MinSince(float now)
    {
        if (_count == 0)
            return 0f;

        float window = WindowSeconds;
        if (window < 0f)
            window = 0f;
        float cutoff = now - window;

        float min = float.MaxValue;
        // Walk from the most recent sample backward. Time is non-decreasing in push order, so
        // the first sample older than the cutoff means every earlier one is out of window too.
        for (int i = 0; i < _count; i++)
        {
            int idx = _head - 1 - i;
            if (idx < 0)
                idx += _time.Length;
            if (_time[idx] < cutoff)
                break;
            if (_value[idx] < min)
                min = _value[idx];
        }
        // At least the newest sample is always within [cutoff, now] when window >= 0.
        return min == float.MaxValue ? _value[(_head - 1 + _time.Length) % _time.Length] : min;
    }
}
