namespace FluidSim.Core;

public static class Gas
{
    public const float Gamma = 1.4f, R = 287f, Cv = R / (Gamma - 1), Cp = Cv + R;
    public const float AmbientPressure = 101325, AmbientTemperature = 293.15f;
    public static State Ambient => State.FromPT(AmbientPressure, AmbientTemperature);
}

public readonly record struct State(float Rho, float M, float E, float Air)
{
    public float U => M / Rho;
    public float P => (Gas.Gamma - 1) * (E - .5f * M * U);
    public float T => P / (Rho * Gas.R);
    public float Y => Air / Rho;
    public float C => MathF.Sqrt(Gas.Gamma * MathF.Max(1, P) / Rho);
    public float H => (E + P) / Rho;
    public static State FromPT(float p, float t, float u = 0, float y = 1)
    {
        float r = p / (Gas.R * t);
        return new(r, r * u, p / (Gas.Gamma - 1) + .5f * r * u * u, r * y);
    }
    public State Reflect() => this with { M = -M };
    public static State operator +(State a, State b) => new(a.Rho+b.Rho,a.M+b.M,a.E+b.E,a.Air+b.Air);
    public static State operator -(State a, State b) => new(a.Rho-b.Rho,a.M-b.M,a.E-b.E,a.Air-b.Air);
    public static State operator *(float s, State a) => new(s*a.Rho,s*a.M,s*a.E,s*a.Air);
    public State Flux => new(M, M * U + P, (E + P) * U, M * Y);
    public static State Rusanov(State a, State b) => .5f * (a.Flux + b.Flux) - (.5f * MathF.Max(MathF.Abs(a.U)+a.C, MathF.Abs(b.U)+b.C)) * (b-a);
}
