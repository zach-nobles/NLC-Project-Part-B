function [Ad, Bd] = continuous_to_discrete(A, B, dt, method)
if nargin == 3
    method = 'Exact';
end
if strcmp(method, 'Exact')
    Ad = expm(A*dt);
    %Bd = (Ad - eye(13))*A^-1*B; doesn't work if A singular
    fun = @(t) expm(A.*t);
    Bd = integral(fun, 0, dt, 'ArrayValued', 1)*B;
elseif strcmp(method, 'Euler')
    Ad = eye(13) + A*dt;
    Bd = B*dt;
else
    error('Put ''Exact'' or ''Euler'' as the method or leave it blank')
end


end