function [ data ] = crg_map_xy2xy( data, crg_xy, iu, iv )
%CRG_MAP_XY2XY Map z-values from one inertial coordinate system to another.
%   [DATA] = CRG_MAP_XY2XY(DATA, CRG_XY, IU, IV) maps z-values of CRG_XY
%   in x/y-coordinates to the inertial x/y-coordinate system of DATA.
%
%   Inputs:
%   DATA        struct array as defined in CRG_INTRO
%   CRG_XY      struct array as defined in CRG_INTRO
%   IU          U index for separate selection (default: full CRG)
%               IU(1): longitudinal start index
%               IU(2): longitudinal end index
%   IV          V index for separate selection (default: full CRG)
%               IV(1): lateral start index
%               IV(2): lateral end index
%
%   Outputs:
%   DATA        struct array as defined in CRG_INTRO
%
%   Examples:
%   data = crg_map_xy2xy(data, crg_xy, iu, iv)
%       Map z-values from crg_xy to data in range of iu and iv.
%
%   See also CRG_INTRO

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_gen_xy2xy 
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

%% default

[nu nv] = size(data.z);

if nargin < 4 || isempty(iv), iv = [1 nv];         end
if nargin < 3 || isempty(iu), iu = [1 nu];         end

if length(iv) < 2, iv = [iv nv];                   end
if length(iu) < 2, iu = [iu nu];                   end

ceps = data.opts.ceps;
ctol = data.opts.ctol;

%% check if already successfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of CRG was not completely successful')
    end
end

if ~isfield(crg_xy, 'ok')
    crg_xy = crg_check(crg_xy);
    if ~isfield(crg_xy, 'ok')
        error('CRG:checkError', 'check of CRG_XY was not completely successful')
    end
end

if length(iu) < 1 || length(iu) > 2
    error('CRG:checkError', 'check of u-spacing was not successful')
end

if length(iv) < 1 || length(iv) > 2
    error('CRG:checkError', 'check of v-spacing was not successful')
end

%% build base uv

ubeg = data.head.ubeg + data.head.uinc*(iu(1)-1);
uend = data.head.ubeg + data.head.uinc*(iu(2)-1);
uinc = data.head.uinc;
u = ubeg:uinc:uend;

if isfield(data.head, 'vinc')
    vmin = data.head.vmin + data.head.vinc*(iv(1)-1);
    vmax = data.head.vmin + data.head.vinc*(iv(2)-1);
    vinc = data.head.vinc;
    v = vmin:vinc:vmax;
else
    v = data.v(iv(1):iv(2));
end

%% get base z-values

[ux, vx] = meshgrid(u, v);

[puv, data] = crg_eval_uv2xy(data, [ux(:), vx(:)]);
z = crg_eval_uv2z(data, [ux(:), vx(:)]);

[xnu, xnv] = size(ux);
z = reshape(z, xnu, xnv);

clear ux vx;

%% rerender crg_uv for equal space

if abs(uinc - crg_xy.head.uinc) > max(ceps*(uinc + crg_xy.head.uinc), ctol)
   tuinc = uinc;
end

if isfield(crg_xy.head, 'vinc')
    if isfield(data.head, 'vinc')
        if abs(data.head.vinc - crg_xy.head.vinc) > max(ceps*(data.head.vinc + crg_xy.head.vinc), ctol)
            tvinc = data.head.vinc;
        end
    else
        tvinc = data.v;
    end
else
    if isfield(data.head, 'vinc')
        tvinc = data.head.vinc;
    elseif abs(data.v - crg_xy.v) > max(ceps*(data.v + crg_xy.v), ctol);
            tvinc = data.v;
    end
end

if exist('tuinc', 'var') && exist('tvinc', 'var')
    crg = crg_rerender(crg_xy, [tuinc tvinc]);
elseif exist('tuinc', 'var')
    crg = crg_rerender(crg_xy, tuinc);
elseif exist('tvinc', 'var')
    crg = crg_rerender(crg_xy, [uinc, tvinc]);
else
    crg = crg_xy;
end

%% find z-values in adding crg

cz = crg_eval_xy2z(crg, puv);
cz = reshape(cz, xnu, xnv);

%% add z-values to base

data.z(iu(1):iu(end),iv(1):iv(end)) = (cz + z)';

%% check

data = crg_check(data);
if ~isfield(data, 'ok')
    error('CRG:checkError', 'check of data was not completely successful')
end

end % function crg_map_xy2xy(data, crg_xy, iu, iv)
