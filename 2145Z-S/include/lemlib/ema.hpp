class EmaFilter
{
public:
    EmaFilter(double alpha) :
        m_alpha(alpha), m_lastOutput(0.0) {}

    double Run(double input)
    {
        m_lastOutput = m_alpha * input + (1 - m_alpha) * m_lastOutput;
        return m_lastOutput;
    }
private:
    double m_alpha;
    double m_lastOutput;
};