function [ data ] = crg_limiter( data, mmlim, iu, iv )
% CRG_LIMITER Limits z-values in OpenCRG data.
%   DATA = CRG_LIMITER( DATA, MMLIM, IU, IV ) limits z-values in OpenCRG data.
%
%   Inputs:
%   DATA        struct array as defined in CRG_INTRO
%   MMLIM       minimal, maximal limits of z values
%               MMLIM(1): minimum limit
%               MMLIM(2): maximum limit
%   IU          U index for separate selection (default: full CRG)
%               IU(1): longitudinal start index
%               IU(2): longitudinal end index
%   IV          V index for separate selection (default: full CRG)
%               IV(1): lateral start index
%               IV(2): lateral end index
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   data = crg_limiter( data, mmlim, iu, iv )
%       Limits z-values.
%   See also CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_limiter.m 
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

if nargin < 4 || isempty(iv), iv = [1 nv];      end
if nargin < 3 || isempty(iu), iu = [1 nu];      end

if length(iv) < 2, iv = [iv nv];                end
if length(iu) < 2, iu = [iu nu];                end
if length(mmlim) <2, mmlim = [mmlim mmlim];     end

%% check if already successfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
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

%% cut off limits

[XI, YI] = meshgrid(u,v);

z = crg_eval_uv2z(data, [XI(:), YI(:)]);

z = reshape(z, size(XI, 1), size(XI, 2));

z(z < mmlim(1)) = mmlim(1);
z(z > mmlim(2)) = mmlim(2);

data.z(iu(1):iu(2), iv(1):iv(end)) = z';

%% delete slope and banking


if isfield(data, 's') || isfield(data, 'b')
    if isfield(data.head, 'zend')
        data.head = rmfield(data.head, 'zend');
    end
    if isfield(data.head, 'aend')
        data.head = rmfield(data.head, 'aend');
    end
end

if isfield(data, 's')
    data = rmfield(data, 's');
    data.head = rmfield(data.head, 'sbeg');
    data.head = rmfield(data.head, 'send');
end

if isfield(data, 'b')
    data = rmfield(data, 'b');
    data.head = rmfield(data.head, 'bbeg');
    data.head = rmfield(data.head, 'bend');
end

%% check

data = crg_single(data);

data = crg_check(data);
if ~isfield(data, 'ok')
    error('CRG:checkError', 'check of DATA was not completely successful')
end

end % function crg_limiter( data, mmlim, iu, iv )
