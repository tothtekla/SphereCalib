classdef PointSet2D
    % 2D point set
    
    properties
        XY (:,2) double % B by 2 point set
    end
    properties (Dependent)
        N (1,1) double % length of XY
    end
    
    methods
        function N = get.N(obj)
            N = size(obj.XY,1);
        end
        function obj = set.N(obj, val)
            if val < obj.N
                obj.XY = obj.XY(1:val, :);
            elseif val > obj.N
                obj.XY = [obj.XY; zeros(val-obj.N,2)]; 
            end
        end
        
        function out = PointSet2D(obj)
            % Constructor
            if nargin == 0
                return; 
            end
            if isa(obj,'PointSet2D')
                out.XY = obj.XY;
            elseif ismatrix(obj) && size(obj,2) == 2
                out.XY = obj;
                if(isempty(obj))
                    warning('The argument is an empty double matrix.')
                end
            else
                error('Invalid constructor arguments provided.');
            end
        end
        
        function sphere = fitSphereShi19(p, r)
            %
            % Estimate ellipse parameters with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % p.XY: 2D ellipse points, N by 2 matrix
            % parametricEllipse: ellipse parameters in order [a, b, ex, ey, theta]
            %
            
            % Coefficient matrix of the three points
            
            %M = [ p.XY(idxs(:), 1).^2 p.XY(idxs(:), 1).*p.XY(idxs(:), 2)  p.XY(idxs(:), 2).^2 ...
            %    p.XY(idxs(:), 1) p.XY(idxs(:), 2) [1; 1; 1]];
            %[~,~,V] = svd(M);

            UV = p.XY(:, :);
            A = [- UV(:,2).^2 - 1, 2*UV(:,1).*UV(:,2), ...
                 2*UV(:,1)      , - UV(:,1).^2 - 1,    ...
                 2*UV(:,2)      , - UV(:,1).^2 - UV(:,2).^2];
            b = -r^2 * (UV(:,1).^2 + UV(:,2).^2 + 1);
            W = A \ b;
            sphere = SphereConverter([sign(W(3))*sqrt(W(1)), sign(W(5))*sqrt(W(4)), sqrt(W(6)), r]);
            %{
            A = W(4) + W(6) - r^2;  % y0^2+z0^2-r^2;
            B = -2*W(2);            % -2*x0*y0;
            C = W(1) + W(6) - r^2;  % x0^2+z0^2-r^2;
            D = -2*W(3);            % -2*x0*z0;
            E = -2*W(5);            % -2*y0*z0;
            F = W(1) + W(4) - r^2;  % x0^2+y0^2-r*r;

            iplicitEllipse = implicitEllipse(A, B, C, D, E, F);
            %}
        end

        function sphere = fitSphereShi19B(p, r)
       
            UV = p.XY(:, :);
            AAA = [- UV(:,2).^2 - 1, 2*UV(:,1).*UV(:,2), ...
                 2*UV(:,1)      , - UV(:,1).^2 - 1,    ...
                 2*UV(:,2)      , - UV(:,1).^2 - UV(:,2).^2];
            b = -r^2 * (UV(:,1).^2 + UV(:,2).^2 + 1);
            W0 = AAA \ b;
            [~, ~, V] = svd(AAA);
            syms k1 k2 k3;
            W = k1*V(:, 4) + k2*V(:, 5) + k3*V(:, 6) + W0;

            eq1 = expand(W(1)*W(4) - W(2)^2);
            eq2 = expand(W(1)*W(6) - W(3)^2);
            eq3 = expand(W(4)*W(6) - W(5)^2);
            eq4 = expand(W(2)*W(3) - W(1)*W(5));
            eq5 = expand(W(2)*W(5) - W(4)*W(3));
            eq6 = expand(W(3)*W(5) - W(6)*W(2));
            
            EQ = [eq1 eq2 eq3 eq4 eq5 eq6];
            M =  zeros(6,10);

            for i = 1:6
                [AA, ~] = coeffs(EQ(i), [k1 k2 k3], 'All');
            
                M(i, :) = double([AA(1,3,3), AA(2,2,3), AA(2,3,2),...
                AA(3,1,3), AA(3,2,2), AA(3,3,1),...
                AA(2,3,3), AA(3,2,3), AA(3,3,2), AA(3,3,3)]);
            end
            
            IB = rref(M);
            B = IB(1:6, 7:10);

            A1 = [-B(1, :); ...
                  -B(2, :); ...
                  -B(3, :); ...
                  1, 0, 0, 0];
            A2 = [-B(2, :); ...
                  -B(4, :); ...
                  -B(5, :); ...
                  0, 1, 0, 0];            
            A3 = [-B(3, :); ...
                  -B(5, :); ...
                  -B(6, :); ...
                  0, 0, 1, 0];
            A_norms = [norm(A1), norm(A2), norm(A3)];
            max_norms = max(A_norms);

            if max_norms == A_norms(1)
                A = A1;
                %disp('A1');
            elseif max_norms == A_norms(2)
                A = A2;
                %disp('A2');
            else
               A = A3;
               %disp('A3');
            end

            [Va,D] = eig(A);
            [~,ind] = sort(diag(D));
            Vs = Va(:,ind);
            Vs = Vs(:, 1) ./ Vs(4, 1);

            W = Vs(1)*V(:, 1) + Vs(2)*V(:, 2) + Vs(3)*V(:, 3) + W0;
            Z = sqrt(W(6));
            sphere = SphereConverter([W(3) / Z, W(3) / Z, Z, r]);
           
        end

        function sphere = fitSphereShi19C(p, r, idxs)
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            UV = p.XY(idxs(:), :);
            AAA = [- UV(:,2).^2 - 1, 2*UV(:,1).*UV(:,2), ...
                 2*UV(:,1)      , - UV(:,1).^2 - 1,    ...
                 2*UV(:,2)      , - UV(:,1).^2 - UV(:,2).^2];
            b = -r^2 * (UV(:,1).^2 + UV(:,2).^2 + 1);
            W0 = AAA \ b;
            [~, ~, V] = svd(AAA);

            M = symbolic_Shi19Coeffs(V, W0);  
            
            IB = rref(M);
            B = IB(1:6, 7:10);

            A1 = [-B(1, :); ...
                  -B(2, :); ...
                  -B(3, :); ...
                  1, 0, 0, 0];
            A2 = [-B(2, :); ...
                  -B(4, :); ...
                  -B(5, :); ...
                  0, 1, 0, 0];            
            A3 = [-B(3, :); ...
                  -B(5, :); ...
                  -B(6, :); ...
                  0, 0, 1, 0];
            A_norms = [norm(A1), norm(A2), norm(A3)];
            max_norms = max(A_norms);

            if max_norms == A_norms(1)
                A = A1;
                %disp('A1');
            elseif max_norms == A_norms(2)
                A = A2;
                %disp('A2');
            else
               A = A3;
               %disp('A3');
            end

            [Va,D] = eig(A);
            ind = find(imag(diag(D)) == 0);
            Vs = Va(:,ind(:));
            Vs = Vs ./ Vs(4, :);

            W = Vs(1)*V(:, 4) + Vs(2)*V(:, 5) + Vs(3)*V(:, 6) + W0;
            Z = sqrt(W(6));
            sphere = SphereConverter([W(3) / Z, W(5) / Z, Z, r]);
            
        end

        function parametricEllipse = fitEllipseShi19(p, idxs, r)
            %
            % Estimate ellipse parameters with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % p.XY: 2D ellipse points, N by 2 matrix
            % parametricEllipse: ellipse parameters in order [a, b, ex, ey, theta]
            %
            
            % Coefficient matrix of the three points
            
            %M = [ p.XY(idxs(:), 1).^2 p.XY(idxs(:), 1).*p.XY(idxs(:), 2)  p.XY(idxs(:), 2).^2 ...
            %    p.XY(idxs(:), 1) p.XY(idxs(:), 2) [1; 1; 1]];
            %[~,~,V] = svd(M);

            UV = p.XY(idxs(:), :);
            A = [- UV(:,2).^2 - 1, 2*UV(:,1).*UV(:,2), ...
                 2*UV(:,1)      , - UV(:,1).^2 - 1,    ...
                 2*UV(:,2)      , - UV(:,1).^2 - UV(:,2).^2];
            b = -r^2 * (UV(:,1).^2 + UV(:,2).^2 + 1);
            W = A \ b;

            A = W(4) + W(6) - r^2;  % y0^2+z0^2-r^2;
            B = -2*W(2);            % -2*x0*y0;
            C = W(1) + W(6) - r^2;  % x0^2+z0^2-r^2;
            D = -2*W(3);            % -2*x0*z0;
            E = -2*W(5);            % -2*y0*z0;
            F = W(1) + W(4) - r^2;  % x0^2+y0^2-r*r;

            ellipse = EllipseConverter([A, B, C, D, E, F]);
            parametricEllipse = ellipse.parametricEllipse;
        end

        function parametricEllipse = fitEllipseShi19B(p, r, idxs)
            %
            % Estimate ellipse parameters with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % p.XY: 2D ellipse points, N by 2 matrix
            % parametricEllipse: ellipse parameters in order [a, b, ex, ey, theta]
            %
            
            % Coefficient matrix of the three points
            
            %M = [ p.XY(idxs(:), 1).^2 p.XY(idxs(:), 1).*p.XY(idxs(:), 2)  p.XY(idxs(:), 2).^2 ...
            %    p.XY(idxs(:), 1) p.XY(idxs(:), 2) [1; 1; 1]];
            %[~,~,V] = svd(M);
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            UV = p.XY(idxs(:), :);
            AAA = [- UV(:,2).^2 - 1, 2*UV(:,1).*UV(:,2), ...
                 2*UV(:,1)      , - UV(:,1).^2 - 1,    ...
                 2*UV(:,2)      , - UV(:,1).^2 - UV(:,2).^2];
            b = -r^2 * (UV(:,1).^2 + UV(:,2).^2 + 1);
            W0 = AAA \ b;
            [~, ~, V] = svd(AAA);
            %{
            syms k1 k2 k3;
            W = k1*V(:, 4) + k2*V(:, 5) + k3*V(:, 6) + W0;

            eq1 = expand(W(1)*W(4) - W(2)^2);
            eq2 = expand(W(1)*W(6) - W(3)^2);
            eq3 = expand(W(4)*W(6) - W(5)^2);
            eq4 = expand(W(2)*W(3) - W(1)*W(5));
            eq5 = expand(W(2)*W(5) - W(4)*W(3));
            eq6 = expand(W(3)*W(5) - W(6)*W(2));
            
            EQ = [eq1 eq2 eq3 eq4 eq5 eq6];
            M =  zeros(6,10);
            Ks_Shi = [k1^2, k1*k2, k1*k3,...
                      k2^2 , k2*k3 , k3^2 ,...
                      k1 , k2 , k3, 1];

            for i = 1:6
                [AA, AT] = coeffs(EQ(i), [k1 k2 k3], 'All');
            
                Ks_check = [AT(1,3,3), AT(2,2,3), AT(2,3,2),...
                AT(3,1,3), AT(3,2,2), AT(3,3,1),...
                AT(2,3,3), AT(3,2,3), AT(3,3,2), AT(3,3,3)];
            
                if(any(Ks_check  ~= Ks_Shi))
                    error('Ks check');
                end
            
                M(i, :) = double([AA(1,3,3), AA(2,2,3), AA(2,3,2),...
                AA(3,1,3), AA(3,2,2), AA(3,3,1),...
                AA(2,3,3), AA(3,2,3), AA(3,3,2), AA(3,3,3)]);
            end
            %} 
            M = symbolic_Shi19Coeffs(V, W0);
            IB = rref(M);
            B = IB(1:6, 7:10);

            A1 = [-B(1, :); ...
                  -B(2, :); ...
                  -B(3, :); ...
                  1, 0, 0, 0];
            A2 = [-B(2, :); ...
                  -B(4, :); ...
                  -B(5, :); ...
                  0, 1, 0, 0];            
            A3 = [-B(3, :); ...
                  -B(5, :); ...
                  -B(6, :); ...
                  0, 0, 1, 0];
            A_norms = [norm(A1), norm(A2), norm(A3)];
            max_norms = max(A_norms);

            if max_norms == A_norms(1)
                A = A1;
                disp('A1');
            elseif max_norms == A_norms(2)
                A = A2;
                disp('A2');
            else
               A = A3;
               disp('A3');
            end

            [Va,D] = eig(A);
            ind = find(imag(diag(D)) == 0);
            Vs = Va(:,ind(:));
            Vs = Vs ./ Vs(4, :);
            
            %k1 = Vs(1);
            %k2 = Vs(2);
            %k3 = Vs(3);
            %WW = double(W);
            W = Vs(1, :).*V(:, 4) + Vs(2, :).*V(:,5) + Vs(3, :).*V(:, 6) + W0;
            
            %{
            eq1 = (W(1,:).*W(4,:) - W(2).^2)
            eq2 = (W(1,:).*W(6,:) - W(3).^2)
            eq3 = (W(4,:).*W(6,:) - W(5).^2)
            eq4 = (W(2,:).*W(3,:) - W(1,:).*W(5,:))
            eq5 = (W(2,:).*W(5,:) - W(4,:).*W(3,:))
            eq6 = (W(3,:).*W(5,:) - W(6,:).*W(2,:))
            %}
            ZZ = abs(W(6,:));
            rsq = r^2;
            
            %{
            AAA
            W
            check = AAA * W
            check2 = b
            %}

            W = W(:, ZZ == max(ZZ));
            
            if all(size(W) == [6,1])

                A = W(4) + W(6) - r^2;  % y0^2+z0^2-r^2;
                B = -2*W(2);            % -2*x0*y0;
                C = W(1) + W(6) - r^2;  % x0^2+z0^2-r^2;
                D = -2*W(3);            % -2*x0*z0;
                E = -2*W(5);            % -2*y0*z0;
                F = W(1) + W(4) - r^2;  % x0^2+y0^2-r*r;
    
                ellipse = EllipseConverter([A, B, C, D, E, F]);
                parametricEllipse = ellipse.parametricEllipse;

            else 
                parametricEllipse = ParametricEllipse();
            end
        end


        function parametricEllipse = fitEllipse(p, idxs)
            %
            % Estimate ellipse parameters with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % p.XY: 2D ellipse points, N by 2 matrix
            % parametricEllipse: ellipse parameters in order [a, b, ex, ey, theta]
            %
            arguments
                p (1,1) PointSet2D
                idxs = 1:p.N;
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no ellipse fitted.');
                parametricEllipse = ParametricEllipse();
                return; 
            end
            if ~isvector(idxs)
                warning('Indices must be a vector, no ellipse fitted.');
                parametricEllipse = ParametricEllipse();
                return; 
            end
            if length(idxs) < 3
                warning('Dimension idxs must be greater or equal than 3, no ellipse fitted.');
                parametricEllipse = ParametricEllipse();
                return; 
            end
            idxN = length(idxs);
            % Add third coordinate z=1 and normalize points
            XY_h = [p.XY(idxs(:), :) ones(idxN, 1)];
            XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;

            % Find the axis drirection and angle of the cone
            wPerCosAlpha = XY_h \ ones(idxN, 1);
            cosAlpha = 1 / norm(wPerCosAlpha);
            w = cosAlpha * wPerCosAlpha;

            % Rotation around the orthogonal vector o with alpha
            sinAlpha = sqrt(1 - cosAlpha^2);
            versinAlpha = 1 - cosAlpha;
            if isequal(w, [0 0 1])
                R = [cosAlpha,   0,    sinAlpha;...
                     0,           1,    0;...
                     -sinAlpha,  0,    cosAlpha]; 
            else
                o = [-w(2) w(1)];
                o = o / norm(o);
                R = [o(1)^2*versinAlpha + cosAlpha, o(1)*o(2)*versinAlpha,         o(2)*sinAlpha;...
                     o(1)*o(2)*versinAlpha,         o(2)^2*versinAlpha + cosAlpha, -o(1)*sinAlpha;...
                     -o(2)*sinAlpha,                o(1)*sinAlpha,                  cosAlpha];
            end
            % Vector from the origin to the direction of the 
            % major ellipse axis ending points
            q_a1 = (R * w);
            q_a2 = (R' * w);
            % Ending points of the major ellipse axes in the image
            a1 = [q_a1(1)/q_a1(3) q_a1(2)/q_a1(3)];
            a2 = [q_a2(1)/q_a2(3) q_a2(2)/q_a2(3)];

            % Ellipse center
            E0 = 0.5*(a1 + a2);
            ex = E0(1); ey = E0(2);
            % Length of the semi-major and semi-minor axis
            a = norm(a2 - a1)/2;
            omega = ex^2 + ey^2 + 1 - a^2;
            b =sqrt((-omega + sqrt(omega^2 + 4*a^2))/2);
            % Rotation angle of the ellipse
            if ey == 0
                theta = 0;
            elseif ex == 0
                theta = 0.5 * pi;
            else
                theta = atan(ey/ex);
            end  

            parametricEllipse = ParametricEllipse([a, b, ex, ey, theta]);
        end
        
        function sphere = fitSphereSun16(p, r, idxs)
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            idxN = length(idxs);
            % Add third coordinate z = sqrt(u^2+v^2)
            A = [p.XY(idxs(:), :) -sqrt(p.XY(idxs(:),1).^2 + p.XY(idxs(:),2).^2 + 1)];
            b = -ones(idxN, 1);
            x = lsqr2(A,b);

            lambda = x(1);
            mu = x(2);
            sigma = x(3);

            s0 = r*[lambda, mu, 1];
            s0_denominator = sqrt(1 + lambda^2 + mu^2 - sigma^2);
            s0 = s0 / s0_denominator;

            sphere = SphereConverter([s0 r]);
        end
        
        function sphere = fitSphere(p, r, idxs)
            %
            % Estimate sphere center with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % XY: 2D ellipse points, N by 2 matrix
            % r: sphere radius
            % s0: sphere center
            %
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            idxN = length(idxs);
            %%timeData = zeros(10, 1);
            %%timerData = tic;
            % Add third coordinate z=1 and normalize points
            XY_h = [p.XY(idxs(:), :) ones(idxN, 1)];
            XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;
            %%timeData(1) = toc(timerData);
            % Find the axis drirection and angle of the cone
            wPerCosAlpha = XY_h \ ones(idxN, 1);
            cosAlpha = 1 / norm(wPerCosAlpha);
            w = cosAlpha * wPerCosAlpha;
            %%timeData(2) = toc(timerData);
            % Direction vec of the cone axis
            d = r / sqrt(1-cosAlpha^2);
            s0 = d * w;            
            %%timeData(9) = toc(timerData);
            sphere = SphereConverter([s0' r]);
            %%timeData(10) = toc(timerData);
            %%timeData
        end
        
        function sphere = fitSphereA(p, r, idxs)
            %
            % Estimate sphere center with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % XY: 2D ellipse points, N by 2 matrix
            % r: sphere radius
            % s0: sphere center
            %
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            if ~isvector(idxs)
                warning('Indices must be a vector, no ellipse fitted.');
                sphere = SphereConverter();
                return; 
            end
            if length(idxs) < 3
                warning('Dimension idxs must be greater or equal than 3, no ellipse fitted.');
                sphere = SphereConverter();
                return; 
            end
            idxN = length(idxs);
            timeData = zeros(10, 1);
            timerData = tic;
            % Add third coordinate z=1 and normalize points
            XY_h = [p.XY(idxs(:), :) ones(idxN, 1)];
            XY_h = sqrt(ones./(sum((XY_h.*XY_h),2)))*ones(1,3).*XY_h;
            timeData(1) = toc(timerData);
            % Find the axis drirection and angle of the cone
            wPerCosAlpha = XY_h \ ones(idxN, 1);
            cosAlpha = 1 / norm(wPerCosAlpha);
            alpha = acos(cosAlpha);
            w = cosAlpha * wPerCosAlpha;
            timeData(2) = toc(timerData);
            % Rotation around the orthogonal vector o with alpha
            sinAlpha = sin(alpha);%sqrt(1 - cosAlpha^2);
            versinAlpha = 1 - cosAlpha;
            if isequal(w, [0 0 1])
                R = [cosAlpha,   0,    sinAlpha;...
                     0,           1,    0;...
                     -sinAlpha,  0,    cosAlpha];
                timeData(3) = toc(timerData);
                R2 = R;
            else
                o = [-w(2) w(1)];
                o = o / norm(o);
                R = [o(1)^2*versinAlpha + cosAlpha, o(1)*o(2)*versinAlpha,         o(2)*sinAlpha;...
                     o(1)*o(2)*versinAlpha,         o(2)^2*versinAlpha + cosAlpha, -o(1)*sinAlpha;...
                     -o(2)*sinAlpha,                o(1)*sinAlpha,                  cosAlpha];
                timeData(3) = toc(timerData);
                sinAlpha = sin(-alpha);
                cosAlpha = cos(-alpha);
                versinAlpha = 1 - cosAlpha;
                R2 = [o(1)^2*versinAlpha + cosAlpha, o(1)*o(2)*versinAlpha,         o(2)*sinAlpha;...
                     o(1)*o(2)*versinAlpha,         o(2)^2*versinAlpha + cosAlpha, -o(1)*sinAlpha;...
                     -o(2)*sinAlpha,                o(1)*sinAlpha,                  cosAlpha];
            end
            timeData(4) = toc(timerData);
            % Vector from the origin to the direction of the 
            % major ellipse axis ending points
            q_a1 = (R * w);
            q_a2 = (R2 * w);
            % Ending points of the major ellipse axes in the image
            a1 = [q_a1(1)/q_a1(3) q_a1(2)/q_a1(3)];
            a2 = [q_a2(1)/q_a2(3) q_a2(2)/q_a2(3)];
            timeData(5) = toc(timerData);
            % Ellipse center
            E0 = 0.5*(a1 + a2);
            ex = E0(1); ey = E0(2);
            % Length of the semi-major and semi-minor axis
            a = norm(a2 - a1)/2;
            omega = ex^2 + ey^2 + 1 - a^2;
            b =sqrt((-omega + sqrt(omega^2 + 4*a^2))/2);
            % Rotation angle of the ellipse
            if ey == 0
                theta = 0;
            elseif ex == 0
                theta = 0.5 * pi;
            else
                theta = atan(ey/ex);
            end  

            %parametricEllipse = ParametricEllipse([a, b, ex, ey, theta]);

            %e->s

            timeData(6) = toc(timerData);
            % Get unit vectors directed to major ellipse axis enging points
            T = [a*cos(theta), -b*sin(theta), ex;...
                 a*sin(theta),  b*cos(theta), ey;...
                 0,               0,          1];
            a1 = (T * [-1, 0, 1]')';
            a2 = ( T * [1 , 0, 1]')';
            q_a1 = a1 / norm(a1);
            q_a2 = a2 / norm(a2);
            timeData(7) = toc(timerData);
            % Angle bisector
            w = (q_a2 + q_a1) / 2;
            w = w / norm(w);
            timeData(8) = toc(timerData);
            % Cone angle and direction
            cos2alpha = q_a1*q_a2';
            alpha = acos(cos2alpha) / 2;
            %d = sqrt(2)*r / sqrt(1-cos2alpha);
            d = r / sin(alpha);
            s0 = d * w;
            timeData(9) = toc(timerData);
            sphere = SphereConverter([s0 r]);
            timeData(10) = toc(timerData);
            timeData
        end
        

        function sphere = fitSphereB(p, r, idxs)
            %
            % Estimate sphere center with only at least 3 points, if the ellipse 
            % is a sphere projection in the image
            %
            % XY: 2D ellipse points, N by 2 matrix
            % r: sphere radius
            % s0: sphere center
            %
            arguments
                p (1,1) PointSet2D
                r (1,1) double
                idxs = 1:p.N;
            end
            if p.N < 3 
                warning('Dimension N must be greater or equal than 3, no sphere fitted.');
                sphere = SphereConverter();
                return; 
            end
            pE = fitEllipse(p, idxs);
            sphere = parametricEllipseO2SphereO(pE, r);
        end
        

        %%{
        function [pointSet, sphere] = detectSphere(pointSet, pointSetMin, r, iterations, ransacThreshold, aMin, aMax)
            %
            % Detect sphere projection (ellipse) contour points in 2D pointset using RANSAC
            % Initial inlier set: 3 random points
            % After then fitting sphere with the inliers
            %
            % points : 2D pointset, N by 2 matrix
            % r: sphere radius
            % iterations : no of RANSAC iterations
            % ransacThreshold: Euclidean distance error threshold of RANSAC
            % aMin, aMax: minimum and maximum value of longer ellipse semi-axis for speed-up
            % inliers: best fitted ellipse inlier points
            % S0 : best fitted sphere center
            %
            arguments
                pointSet (:,1) PointSet2D
                pointSetMin (:,1) PointSet2D
                r (1,1) double = 0.25
                iterations (1,1) double = 10000
                ransacThreshold (1,1) double = 5
                aMin (1,1) double = 0.01
                aMax (1,1) double = 1000
            end
            sphere = SphereConverter.empty(length(pointSet), 0);
            fig = figure;
            f = waitbar(0,'Detecting sphere');
            for i = 2 : length(pointSet)
                inlierIdxsBest = [];
                waitText = strcat({'Detecting '}, num2str(i), '-th sphere...');
                waitbar(i/length(pointSet), f, waitText{1});
                if pointSetMin(i).N < 3
                    warningMessage = strcat({'Few points in the point set: '}, num2str(i));
                    warning(warningMessage{1});
                    %skip
                else
                    for j = 1 : iterations
                        % generate 3 random point 
                        inlierIdxs = randperm(pointSetMin(i).N, 3);
                        % approximate sphere projection parameters with 3 points  
                        ellipseParam = fitEllipse(pointSetMin(i), inlierIdxs); %, r);
                        % optional speed-up: longer ellipse semi-axis smaller than aMin or greater than aMax is assumed unrealistic
                        if ellipseParam.a > aMin && ellipseParam.a < aMax
                            % label points
                            % distance of the points from the ellipse with numerical approximation
                            distances = pointEllipseDistance(pointSetMin(i), ellipseParam.a, ellipseParam.b,...
                                        ellipseParam.e0(1), ellipseParam.e0(2), ellipseParam.theta);
                            % if the point fits, save the index
                            inlierIdxs = find(distances < ransacThreshold);
                            % save the best model
                            if(length(inlierIdxs) > length(inlierIdxsBest))
                                inlierIdxsBest = inlierIdxs;
                                % dev mode: show inliers
                                plotEllipseWithDist(pointSetMin(i), pointSetMin(i).XY(inlierIdxsBest, :), ...
                                    distances, fig);
                            end
                        end
                    end
                    ellipseParamBest = fitEllipse(pointSetMin(i), inlierIdxsBest);%, r);
                    distances = pointEllipseDistance(pointSet(i), ellipseParamBest.a, ellipseParamBest.b, ...
                                ellipseParamBest.e0(1), ellipseParamBest.e0(2), ellipseParamBest.theta);
                    % if the point fits, save the index
                    inlierIdxs = find(distances < ransacThreshold);
                    % dev mode: show inliers
                    plotEllipseWithMin(pointSet(i), pointSetMin(i),...
                        pointSet(i).XY(inlierIdxs, :), pointSetMin(i).XY(inlierIdxsBest, :), fig, distances);
                    % re-fit a sphere
                    pointSet(i).XY = pointSet(i).XY(inlierIdxs, :);
                    sphere(i) = fitSphere(pointSet(i), r);
                end
            end
            waitbar(1,f,'Sphere detection finished!');
            close(f);
            %close(fig);
        end
        %}

    function dist = pointEllipseDistance(pointSet, a, b, ex, ey, theta)
            %
            % Point-ellipse distance with numerical approximation
            % Based on: https://www.iquilezles.org/www/articles/ellipsoids/ellipsoids.htm
            %
            % pointSet : 2D points
            % [a, b, ex, ey, theta] : ellipse parameters
            % dist : estimated distance
            %
            arguments
                pointSet (1,1) PointSet2D
                a (1,1) double
                b (1,1) double 
                ex (1,1) double
                ey (1,1) double
                theta (1,1) double
            end
            ab = [a b];
            px0 = cos(theta)*(pointSet.XY(:,1) - ex) + sin(theta)*(pointSet.XY(:,2) - ey);
            py0 = - sin(theta)*(pointSet.XY(:,1) - ex)  + cos(theta)*(pointSet.XY(:,2) - ey);
            p = [abs(px0)  abs(py0)];
            k0 = p./ab;
            k0 = sqrt(sum(k0.*k0, 2));
            k1 = p./(ab.*ab);
            k1 = sqrt(sum(k1.*k1, 2));
            dist = abs(k0.*(k0-1)./k1); 
        end

        function plotEllipseWithMin(pointSet, pointSetMin, ell, ellMin, fig, dist)
            arguments
                pointSet (1,1) PointSet2D
                pointSetMin (1,1) PointSet2D
                ell (:,2) double
                ellMin (:,2) double
                fig (1,1) matlab.ui.Figure
                dist (:, 1) double
            end
            % TODO: with ellipse contour
            clf(fig);
            hold on;
            axis equal;
            if isempty(dist)
                scatter(pointSet.XY(:,1), -pointSet.XY(:,2),'filled');
            else
                scatter(pointSet.XY(:,1), -pointSet.XY(:,2), 50, dist, 'filled');
            end
            scatter(pointSetMin.XY(:,1), -pointSetMin.XY(:,2),'filled');
            scatter(ell(:,1), -ell(:,2), '+');
            scatter(ellMin(:,1), -ellMin(:,2), 'x');
            legend('Points', 'PointsMin', 'Ellipse', 'EllipseMin');
        end

        function plotEllipseWithDist(pointSet, ell, dist, fig)
            arguments
                pointSet (1,1) PointSet2D
                ell (:,2) double
                dist (:,1) double
                fig (1,1) matlab.ui.Figure
            end
            % TODO: with ellipse contour
            clf(fig);
            hold on;
            scatter(pointSet.XY(:,1), -pointSet.XY(:,2), 50, dist, 'filled');
            scatter(ell(:,1), -ell(:,2), 50, '*');
            legend('Points', 'Ellipse');
        end
    end

    methods (Static)
        function M = symbolic_Shi19Coeffs(V, W0)  
            v4_1 = V(1, 4);
            v4_2 = V(2, 4);
            v4_3 = V(3, 4);
            v4_4 = V(4, 4);
            v4_5 = V(5, 4);
            v4_6 = V(6, 4);
            
            v5_1 = V(1, 5);
            v5_2 = V(2, 5);
            v5_3 = V(3, 5);
            v5_4 = V(4, 5);
            v5_5 = V(5, 5);
            v5_6 = V(6, 5);
            
            v6_1 = V(1, 6);
            v6_2 = V(2, 6);
            v6_3 = V(3, 6);
            v6_4 = V(4, 6);
            v6_5 = V(5, 6);
            v6_6 = V(6, 6);
            
            w0_1 = W0(1);
            w0_2 = W0(2);
            w0_3 = W0(3);
            w0_4 = W0(4);
            w0_5 = W0(5);
            w0_6 = W0(6);
            
            M = [[ - v4_2^2 + v4_1*v4_4,           v4_1*v5_4 - 2*v4_2*v5_2 + v4_4*v5_1,           v4_1*v6_4 - 2*v4_2*v6_2 + v4_4*v6_1,  - v5_2^2 + v5_1*v5_4,           v5_1*v6_4 - 2*v5_2*v6_2 + v5_4*v6_1,  - v6_2^2 + v6_1*v6_4,           v4_1*w0_4 - 2*v4_2*w0_2 + v4_4*w0_1,           v5_1*w0_4 - 2*v5_2*w0_2 + v5_4*w0_1,           v6_1*w0_4 - 2*v6_2*w0_2 + v6_4*w0_1,  - w0_2^2 + w0_1*w0_4]
            [ - v4_3^2 + v4_1*v4_6,           v4_1*v5_6 - 2*v4_3*v5_3 + v4_6*v5_1,           v4_1*v6_6 - 2*v4_3*v6_3 + v4_6*v6_1,  - v5_3^2 + v5_1*v5_6,           v5_1*v6_6 - 2*v5_3*v6_3 + v5_6*v6_1,  - v6_3^2 + v6_1*v6_6,           v4_1*w0_6 - 2*v4_3*w0_3 + v4_6*w0_1,           v5_1*w0_6 - 2*v5_3*w0_3 + v5_6*w0_1,           v6_1*w0_6 - 2*v6_3*w0_3 + v6_6*w0_1,  - w0_3^2 + w0_1*w0_6]
            [ - v4_5^2 + v4_4*v4_6,           v4_4*v5_6 - 2*v4_5*v5_5 + v4_6*v5_4,           v4_4*v6_6 - 2*v4_5*v6_5 + v4_6*v6_4,  - v5_5^2 + v5_4*v5_6,           v5_4*v6_6 - 2*v5_5*v6_5 + v5_6*v6_4,  - v6_5^2 + v6_4*v6_6,           v4_4*w0_6 - 2*v4_5*w0_5 + v4_6*w0_4,           v5_4*w0_6 - 2*v5_5*w0_5 + v5_6*w0_4,           v6_4*w0_6 - 2*v6_5*w0_5 + v6_6*w0_4,  - w0_5^2 + w0_4*w0_6]
            [v4_2*v4_3 - v4_1*v4_5, v4_2*v5_3 + v4_3*v5_2 - v4_1*v5_5 - v4_5*v5_1, v4_2*v6_3 + v4_3*v6_2 - v4_1*v6_5 - v4_5*v6_1, v5_2*v5_3 - v5_1*v5_5, v5_2*v6_3 + v5_3*v6_2 - v5_1*v6_5 - v5_5*v6_1, v6_2*v6_3 - v6_1*v6_5, v4_2*w0_3 + v4_3*w0_2 - v4_1*w0_5 - v4_5*w0_1, v5_2*w0_3 + v5_3*w0_2 - v5_1*w0_5 - v5_5*w0_1, v6_2*w0_3 + v6_3*w0_2 - v6_1*w0_5 - v6_5*w0_1, w0_2*w0_3 - w0_1*w0_5]
            [v4_2*v4_5 - v4_3*v4_4, v4_2*v5_5 - v4_3*v5_4 - v4_4*v5_3 + v4_5*v5_2, v4_2*v6_5 - v4_3*v6_4 - v4_4*v6_3 + v4_5*v6_2, v5_2*v5_5 - v5_3*v5_4, v5_2*v6_5 - v5_3*v6_4 - v5_4*v6_3 + v5_5*v6_2, v6_2*v6_5 - v6_3*v6_4, v4_2*w0_5 - v4_3*w0_4 - v4_4*w0_3 + v4_5*w0_2, v5_2*w0_5 - v5_3*w0_4 - v5_4*w0_3 + v5_5*w0_2, v6_2*w0_5 - v6_3*w0_4 - v6_4*w0_3 + v6_5*w0_2, w0_2*w0_5 - w0_3*w0_4]
            [v4_3*v4_5 - v4_2*v4_6, v4_3*v5_5 - v4_2*v5_6 + v4_5*v5_3 - v4_6*v5_2, v4_3*v6_5 - v4_2*v6_6 + v4_5*v6_3 - v4_6*v6_2, v5_3*v5_5 - v5_2*v5_6, v5_3*v6_5 - v5_2*v6_6 + v5_5*v6_3 - v5_6*v6_2, v6_3*v6_5 - v6_2*v6_6, v4_3*w0_5 - v4_2*w0_6 + v4_5*w0_3 - v4_6*w0_2, v5_3*w0_5 - v5_2*w0_6 + v5_5*w0_3 - v5_6*w0_2, v6_3*w0_5 - v6_2*w0_6 + v6_5*w0_3 - v6_6*w0_2, w0_3*w0_5 - w0_2*w0_6]];
        end
    end
end

