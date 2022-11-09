classdef EKF < handle

    properties (Access = public)
       sampleRate = 1024;
       Quaternion = [1,0,0,0];
       gyroNoise = 0.3^2;
       accelNoise = 0.5^2;
       magNoise = 0.8^2;
       P = eye(4);
       magAngle = 0.5935;
    end

    methods
        function obj = EKF(varargin)
            for i = 1 : 2 : nargin
                if  strcmp(varargin{i}, 'SampleRate'), obj.sampleRate = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'GyroNoise'), obj.gyroNoise = varargin{i+1};
                elseif  strcmp(varargin{i}, 'AccelNoise'), obj.accelNoise = varargin{i+1};
                elseif  strcmp(varargin{i}, 'MagNoise'), obj.magNoise = varargin{i+1};
                elseif  strcmp(varargin{i}, 'MagAngle'), obj.magAngle = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end

        function obj = Update(obj,accel,gyro,mag)

            if(norm(accel) == 0), return; end	% handle NaN
            accel = accel/norm(accel);

            if(norm(mag) == 0), return; end	% handle NaN
            mag = mag/norm(mag);

            q = obj.Quaternion;
            dt = 1/obj.sampleRate;
            g = [0,0,1]';
            %r = [cos(obj.magAngle),0,sin(obj.magAngle)]';
            m = quatmultiply(q, quatmultiply([0, mag], quatconjugate(q)));
            r = [norm([m(2), m(3)]), 0, m(4)]';
            r = r/norm(r);

            q = [q(1)-dt/2*gyro(1)*q(2)-dt/2*gyro(2)*q(3)-dt/2*gyro(3)*q(4);
                q(2)+dt/2*gyro(1)*q(1)-dt/2*gyro(2)*q(4)+dt/2*gyro(3)*q(3);
                q(3)+dt/2*gyro(1)*q(4)+dt/2*gyro(2)*q(1)-dt/2*gyro(3)*q(2);
                q(4)-dt/2*gyro(1)*q(3)+dt/2*gyro(2)*q(2)+dt/2*gyro(3)*q(1)];

            F = [1, -dt/2*gyro(1), -dt/2*gyro(2), -dt/2*gyro(3);
                dt/2*gyro(1), 1, dt/2*gyro(3), -dt/2*gyro(2);
                dt/2*gyro(2), -dt/2*gyro(3), 1, dt/2*gyro(1);
                dt/2*gyro(3), dt/2*gyro(2), -dt/2*gyro(1), 1];

            W = dt/2*[-q(2), -q(3), -q(4);
                q(1), -q(4), q(3);
                q(4), q(1), -q(2);
                -q(3), q(2), q(1)];

            sigmaGyro = obj.gyroNoise*eye(3);

            Q = W*sigmaGyro*W';

            obj.P = F*obj.P*F' + Q;

            z = 2*[g(1)*(1/2-q(3)^2-q(4)^2)+g(2)*(q(1)*q(4)+q(2)*q(3))+g(3)*(q(2)*q(4)-q(1)*q(3));
                g(1)*(q(2)*q(3)-q(1)*q(4))+g(2)*(1/2-q(2)^2-q(4)^2)+g(3)*(q(1)*q(2)+q(3)*q(4));
                g(1)*(q(1)*q(3)+q(2)*q(4))+g(2)*(q(3)*q(4)-q(1)*q(2))+g(3)*(1/2-q(2)^2-q(3)^2);
                r(1)*(1/2-q(3)^2-q(4)^2)+r(2)*(q(1)*q(4)+q(2)*q(3))+r(3)*(q(2)*q(4)-q(1)*q(3));
                r(1)*(q(2)*q(3)-q(1)*q(4))+r(2)*(1/2-q(2)^2-q(4)^2)+r(3)*(q(1)*q(2)+q(3)*q(4));
                r(1)*(q(1)*q(3)+q(2)*q(4))+r(2)*(q(3)*q(4)-q(1)*q(2))+r(3)*(1/2-q(2)^2-q(3)^2)];

            H = 2*[g(2)*q(4)-g(3)*q(3), g(2)*q(3)+g(3)*q(4), -2*g(1)*q(3)+g(2)*q(2)-g(3)*q(1), -2*g(1)*q(4)+g(2)*q(1)+g(3)*q(2);
                -g(1)*q(4)+g(3)*q(2), g(1)*q(3)-2*g(2)*q(2)+g(3)*q(1), g(1)*q(2)+g(3)*q(4), -g(1)*q(1)-2*g(2)*q(4)+g(3)*q(3);
                g(1)*q(3)-g(2)*q(2), g(1)*q(4)-g(2)*q(1)-2*g(3)*q(2), g(1)*q(1)+g(2)*q(4)-2*g(3)*q(3), g(1)*q(2)+g(2)*q(3);
                r(2)*q(4)-r(3)*q(3), r(2)*q(3)+r(3)*q(4), -2*r(1)*q(3)+r(2)*q(2)-r(3)*q(1), -2*r(1)*q(4)+r(2)*q(1)+r(3)*q(2);
                -r(1)*q(4)+r(3)*q(2), r(1)*q(3)-2*r(2)*q(2)+r(3)*q(1), r(1)*q(2)+r(3)*q(4), -r(1)*q(1)-2*r(2)*q(4)+r(3)*q(3);
                r(1)*q(3)-r(2)*q(2), r(1)*q(4)-r(2)*q(1)-2*r(3)*q(2), r(1)*q(1)+r(2)*q(4)-2*r(3)*q(3), r(1)*q(2)+r(2)*q(3)];

            R = [obj.accelNoise*eye(3),zeros(3);
                zeros(3),obj.magNoise*eye(3)];

            K = obj.P*H'/(H*obj.P*H' + R);

            q = q + K*([accel,mag]' - z);
            obj.P = (eye(4) - K*H)*obj.P;

            obj.Quaternion = q'/norm(q');

        end
    end
end