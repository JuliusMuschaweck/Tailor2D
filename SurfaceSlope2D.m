%% SurfaceSlope2D
% Find slope of 2D optical surface matching a pair of 2D rays
%% Syntax
% |[slope, doReflect] = SurfaceSlope2D(indir, outdir, leftright_flag)|
%% Description
% |slope = SurfaceSlope2D(indir, outdir, refract_flag, leftright_flag)| finds the slope of an optical 
% surface such that the ray with direction |indir| will be refracted or reflected to continue with 
% direction |outdir|.
%
% |[slope, doReflect] = SurfaceSlope2D(indir, outdir, leftright_flag)| additionally informs the caller whether
% |indir| and |outdir| point towards oppposite sides of |slope|, i.e. they reflect.
%% Examples
%% Input Arguments
% * |indir|: Incoming ray direction. 2D column vector. The length is the refractive index.
% * |outdir|: Outgoing ray direction. 2D column vector. The length is the refractive index.
% * |leftright_flag|: Scalar logical. When true, |slope| points towards the left of |indir|, when false,
% to the right.
%% Output Arguments
% * |slope|: Surface slope (tangent). 2D column vector of length 1.
% * |doReflect|: Scalar logical. True if |indir| and |outdir| point towards oppposite sides of |slope|.
%% Algorithm
% Since the ray directions have the refractive index as length, the surface normal is simply |indir -
% outdir|. This works both for refraction and reflection. Also proceeds happily in the unphysical case of
% unwanted total internal reflection. The |slope| is then a unit vector perpendicular to the surface
% normal. Reflection is detected by comparing the signs of the 2D cross products of ray directions and
% slope. 
%
% <html>
% <a href="file://C:/Users/jm/Documents/BUSINESS/Software_JM/Matlab/Include/RayTrace2D/Tailor2D/html/Cross2D.html"> Cross2D </a>
% </html>
%
% <html>
% <a href="file:./Cross2D.html"> Cross2D </a>
% </html>
%
% <file:Cross2D.html Cross2D>
function [slope, doReflect] = SurfaceSlope2D(indir, outdir, leftright_flag)
    arguments
        indir (2,1) double
        outdir (2,1) double
        leftright_flag (1,1) logical
    end
    normal = indir - outdir; % yes, works for refraction and reflection
    slope = [-normal(2); normal(1)]; % 90° to the left
    cis = Cross2D(indir, slope);
    cos = Cross2D(outdir, slope);
    doReflect = (cis * cos <= 0); 
    % cis * cos < 0: in and out on different sides of slope -> reflection, shouldn't happen for
    % refraction, i.e. when lengths of indir and outdir are different, but function computes a result
    % even for this nonphysical situation.
    % cis == 0 xor cos == 0 for grazing incidence: Refraction at TIR limit
    % cis == cos == 0 for indir = outdir: grazing incidence reflection
    if (cis < 0) == logical(leftright_flag)
        slope = - slope;
    end
    slope = Unit2D(slope);
end