using Xunit;

namespace BetterIk;

public class MathBridgeTests
{
    [Fact]
    public void Vector3_RoundTrips_ThroughNumerics()
    {
        var original = new Vector3(1.5f, -2.25f, 3.75f);

        var roundTripped = original.ToNumerics().ToSandbox();

        Assert.Equal(original.x, roundTripped.x, 5);
        Assert.Equal(original.y, roundTripped.y, 5);
        Assert.Equal(original.z, roundTripped.z, 5);
    }

    [Fact]
    public void Quaternion_RoundTrips_ThroughRotation()
    {
        var original = System.Numerics.Quaternion.CreateFromAxisAngle(new System.Numerics.Vector3(0, 1, 0), 0.7f);

        var roundTripped = original.ToSandbox().ToNumerics();

        Assert.Equal(original.X, roundTripped.X, 5);
        Assert.Equal(original.Y, roundTripped.Y, 5);
        Assert.Equal(original.Z, roundTripped.Z, 5);
        Assert.Equal(original.W, roundTripped.W, 5);
    }

    [Fact]
    public void Rotation_RoundTrips_ThroughNumerics()
    {
        var original = new Rotation(0.1f, 0.2f, 0.3f, 0.9f);

        var roundTripped = original.ToNumerics().ToSandbox();

        Assert.Equal(original.x, roundTripped.x, 5);
        Assert.Equal(original.y, roundTripped.y, 5);
        Assert.Equal(original.z, roundTripped.z, 5);
        Assert.Equal(original.w, roundTripped.w, 5);
    }

    [Fact]
    public void Identity_MapsToIdentity()
    {
        var sandboxIdentity = Rotation.Identity;
        var numericsIdentity = sandboxIdentity.ToNumerics();

        Assert.Equal(System.Numerics.Quaternion.Identity.X, numericsIdentity.X, 5);
        Assert.Equal(System.Numerics.Quaternion.Identity.Y, numericsIdentity.Y, 5);
        Assert.Equal(System.Numerics.Quaternion.Identity.Z, numericsIdentity.Z, 5);
        Assert.Equal(System.Numerics.Quaternion.Identity.W, numericsIdentity.W, 5);
    }
}
