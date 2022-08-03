function TR = Tnormalise(T)
% Normalise a transform matrix or Rotation matrix after an infinitesimal
% update in rotation angles TR = Tnormalise(T)
%
%   RN = trnorm(R) is guaranteed to be a proper orthogonal matrix rotation
%   matrix (3x3) which is "close" to the non-orthogonal matrix R (3x3). If R
%   = [N,O,A] the O and A vectors are made unit length and the normal vector
%   is formed from N = O x A, and then we ensure that O and A are orthogonal
%   by O = A x N.
%  
%   TN = trnorm(T) as above but the rotational submatrix of the homogeneous
%   transformation T (4x4) is normalised while the translational part is
%   passed unchanged.
%  
%   If R (3x3xK) or T (4x4xK) represent a sequence then RN and TN have the
%   same dimension and normalisation is performed on each plane.
%  
%   Notes::
%   - Only the direction of A (the z-axis) is unchanged.
%   - Used to prevent finite word length arithmetic causing transforms to 
%     become `unnormalized'.

    %assert(ishomog(T) || isrot(T), 'RTB:trnorm:badarg', 'expecting 3x3xN or 4x4xN hom xform');
    
    if ndims(T) == 3
        % recurse for transform sequence
        nd = size(T, 3);
        r = zeros(4,4,nd);
        for i=1:nd
            TR(:,:,i) = trnorm(T(:,:,i));
        end
        return
    end
    
    n = T(1:3,1); o = T(1:3,2); a = T(1:3,3);
    n = cross(o, a);         % N = O x A
    o = cross(a, n);         % O = A x N
    R = [unit(n) unit(o) unit(a)];
    
    if ishomog(T)
        TR = rt2tr( R, T(1:3,4) );
    elseif isrot(T)
        TR = R;
    end