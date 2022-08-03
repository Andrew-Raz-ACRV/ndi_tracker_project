function A = register_ideal_maps(pa,pb)
%Solves Transform A in A*pb = pa
% for point column pa and pb = [x; y; z];

%Ra*pb + xa - pa = 0
%So xa = pa - Ra*pb
%Get the average to find xa...
pa_av = mean(pa,2);
pb_av = mean(pb,2);

%Now define Data Matrices:
Ma = (pa - pa_av);
Mb = (pb - pb_av);

%Ra*Mb = Ma, rank(Mb) == 3
[U,S,V] = svd(Ma*Mb'/(Mb*Mb'));
%disp(norm(eye(3)-S))

%solve rotation
Ra = U*V;

%solve translation
xa = pa_av - Ra*pb_av;

A = [Ra xa; 0 0 0 1];
end

% %Ra*pb + xa - pa = 0
% %So xa = pa - Ra*pb
% %Get the average to find xa...
% pa_av = mean(pa,2);
% pb_av = mean(pb,2);
% 
% %Now define Data Matrices:
% Ma = (pa - pa_av);
% Mb = (pb - pb_av);
% 
% %Ra*Mb = Ma, rank(Mb) == 3
% [U,S,V] = svd(Ma*Mb'/(Mb*Mb'));
% %disp(norm(eye(3)-S))
% 
% %solve rotation
% Ra = U*V;
% 
% %solve translation
% xa = pa_av - Ra*pb_av;