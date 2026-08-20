#include <Arduino.h>
#include <ArduinoEigenDense.h>

//hardcoding discretized matrices to make it easier for esp

Eigen::Matrix4f Ad;
Eigen::Vector4f Bd;
Eigen::Matrix<float, 2, 4> C;
Eigen::Matrix4f Qk;
Eigen::Matrix2f Rk;
Eigen::RowVector4f K;

//for estimation: 
Eigen::Vector4f x_hat = Eigen::Vector4f::Zero();
Eigen::Matrix4f P = Eigen::Matrix4f::Identity();

float u_prev = 0.0f;
float u_max = 10.0f;

void kalmanStep(
    Eigen::Vector4f& x_hat,
    Eigen::Matrix4f& P,
    float u,
    const Eigen::Vector2f& y)
{
    Eigen::Vector4f x_pred = Ad * x_hat + Bd * u;
    Eigen::Matrix4f P_pred = Ad * P * Ad.transpose() + Qk;
    
    Eigen::Vector2f residual = y - C * x_pred;
    Eigen::Matrix2f S = C * P_pred * C.transpose() + Rk;

    Eigen::Matrix<float, 4, 2> Kk = P_pred * C.transpose() * S.inverse();

    x_hat = x_pred + Kk * residual;

    P = (Eigen::Matrix4f::Identity() - Kk * C) * P_pred;

} 


void setup()
{
    Ad <<
    1.0f, 0.01f, -4.90588223e-05f, -1.63517644e-07f,
    0.0f, 1.0f, -9.81352904e-03f, -4.90588223e-05f,
    0.0f, 0.0f, 1.00107929f, 1.00035974e-02f,
    0.0f, 0.0f, 2.15897639e-01f, 1.00107929f;

    Bd <<
        5.00008176e-05f,
        1.00003270e-02f,
        -1.00017986e-04f,
        -2.00071948e-02f;

    C <<
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f;

    Qk <<
        1e-6f, 0.0f, 0.0f, 0.0f,
        0.0f, 1e-4f, 0.0f, 0.0f,
        0.0f, 0.0f, 1e-6f, 0.0f,
        0.0f, 0.0f, 0.0f, 1e-4f;

    Rk <<
        4e-6f, 0.0f,
        0.0f, 1.21846968e-05f;
    
    K << 
        -3.16227766f, -5.5397091f, -56.84055809f, -10.86364348f;

    Serial.begin(115200);
}


void loop()
{
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.startsWith("MEAS,"))
        {
            int firstComma = line.indexOf(',');
            int secondComma = line.indexOf(',', firstComma + 1);

            if (secondComma != -1)
            {
                float position =
                    line.substring(firstComma + 1, secondComma).toFloat();

                float angle =
                    line.substring(secondComma + 1).toFloat();

                Eigen::Vector2f y;
                y << position, angle;

                kalmanStep(x_hat, P, u_prev, y);
                float u = -(K * x_hat)(0);
                u = constrain(u, -u_max, u_max);
                u_prev = u;

                Serial.print("DATA,");
                Serial.print(x_hat(0), 6);
                Serial.print(",");
                Serial.print(x_hat(1), 6);
                Serial.print(",");
                Serial.print(x_hat(2), 6);
                Serial.print(",");
                Serial.print(x_hat(3), 6);
                Serial.print(",");
                Serial.println(u, 6);
            }
        }
    }
}