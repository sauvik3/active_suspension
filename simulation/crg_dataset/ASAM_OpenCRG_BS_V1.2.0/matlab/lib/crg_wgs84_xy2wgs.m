function [wgs, data] = crg_wgs84_xy2wgs(data, pxy)
%CRG_WGS84_XY2WGS Transform points in x/y-coordinates to WGS-84 coordinates.
%   [WGS, DATA] = CRG_WGS84_XY2WGS(DATA, PXY) transforms points given in local
%   x/y-coordinates to WGS-84 coordinates using the provided OpenCRG data for
%   reference
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%   PXY     (np, 2) array of points in xy system
%
%   Outputs:
%   WGS     (np, 2) array of latitude/longitude (north/east) wgs84
%           coordinate pairs (in degrees)
%   DATA    struct array as defined in CRG_INTRO.
%
%   Examples:
%   [wgs, data] = crg_wgs84_xy2wgs(data, pxy)
%   transforms pxy points to WGS84 coordinates
%
%   See also CRG_INTRO, CRG_WGS84_WGSXY2WGS.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_wgs84_xy2wgs.m 
% author:                ASAM e.V.
%
%
% C by ASAM e.V., 2020
% Any use is limited to the scope described in the license terms.
% The license terms can be viewed at www.asam.net/license
%
% More Information on ASAM OpenCRG can be found here:
% https://www.asam.net/standards/detail/opencrg/
%
% *****************************************************************

%% check if already successfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
end

%% simplify data access

crgeps = data.opts.ceps;
crgtol = data.opts.ctol;
crgwgs = data.opts.cwgs;

xbeg = data.head.xbeg;
ybeg = data.head.ybeg;

np = size(pxy, 1);

%% check if MPRO data is available, evaluate

if isfield(data, 'mpro') % mapping projection data is available
    xyz = zeros(np, 3);
    xyz(:, 1:2) = pxy;
    
    llh = crg_eval_xyz2llh(data, xyz);
    
    wgs = llh(:, 1:2) * 180/pi;
    
    return
end

%% check if WGS-84 end is available, evaluate

if isfield(data.head, 'eend') % start and end are both defined
    wgs1 = [data.head.nbeg data.head.ebeg];
    wgs2 = [data.head.nend data.head.eend];
    pxy1 = [data.head.xbeg data.head.ybeg];
    pxy2 = [data.head.xend data.head.yend];
    wgs = crg_wgs84_wgsxy2wgs(wgs1, wgs2, pxy1, pxy2, pxy, crgeps, crgwgs);
elseif isfield(data.head, 'ebeg') % only start is defined
    wgs = zeros(size(pxy));
    for i = 1:np
        dist = sqrt((pxy(i,1)-xbeg)^2 + (pxy(i,2)-ybeg)^2);
        if dist <= crgtol
            wgs(i,1) = data.head.nbeg;
            wgs(i,2) = data.head.ebeg;
        else
            error('CRG:wgs84Error', 'insufficient WGS84 information available')
        end
    end
else
    error('CRG:wgs84Error', 'no WGS84 information available')
end

end
