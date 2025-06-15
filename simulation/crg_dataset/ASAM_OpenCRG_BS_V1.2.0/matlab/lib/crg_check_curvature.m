function [data, ierr, idxArr] = crg_check_curvature(data, ierr)
% CRG_CHECK_CURVATURE Check OpenCRG curvature data.
%   [DATA] = CRG_CHECK_CURVATURE(DATA) checks reference line curvature in `data`
%   globally and locally. The global curvature check fails if two or more
%   lateral cuts intersect inside the road limits. In this case, the local
%   curvature check still succeeds, if such an intersection falls into a region
%   of NaN values.

%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%   IERR    number of previous errors
%
%   Outputs:
%   DATA    is a checked, purified, and eventually completed version of
%           the function input argument DATA
%   IERR    summed up number of errors
%   IDXARR  array containing start and end index, where local error occured
%           if no local error occured an empty error is returned
%
%   Examples:
%   data = crg_check_curvature(data)
%       Checks CRG reference line curvature.
%
%   See also CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_check_curvature.m 
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


%% some local variables

crgeps = data.opts.ceps;
icerr = 0;
idxArr = [];

%% check for rc field (reference line curvature)

if ~isfield(data, 'rc')
    error('CRG:checkError', 'DATA.rc missing')
end

%% global curvature check


cmin = min(data.rc);
cmax = max(data.rc);
if abs(cmax) > crgeps
    if 1/cmax <= data.head.vmax && 1/cmax >= data.head.vmin
        warning('CRG:checkWarning', 'global curvature check failed - center of max. reference line curvature=%d inside road limits', cmax)
        icerr = icerr + 1;
    end
end
if abs(cmin) > crgeps
    if 1/cmin <= data.head.vmax && 1/cmin >= data.head.vmin
        warning('CRG:checkWarning', 'global curvature check failed - center of min. reference line curvature=%d inside road limits', cmin)
        icerr = icerr + 1;
    end
end

ierr = ierr + icerr;

%% local curvature check

if isfield(data.opts, 'wcvl') && data.opts.wcvl > 0 && icerr > 0
    % set temp ok
    data.ok = 0;
    
    % reference line points
    uges=data.head.ubeg:data.head.uinc:data.head.uend;
    uinc=data.head.uinc;
    %%

    % indices l/r curvature
    vek_rc=[data.rc(1),data.rc,data.rc(end)];
    idx_right=vek_rc< 0;
    idx_left =vek_rc>=0;

    % min max v values from curvature radius
    min_max_v = NaN(size(vek_rc));
    min_max_v(idx_right)=ceil(1./vek_rc(idx_right)./data.head.uinc).*data.head.uinc; 
    min_max_v(idx_left) =floor(1./vek_rc(idx_left)./data.head.uinc).*data.head.uinc;

    % max grid cells array
    rightNaNBorder=data.head.vmin.*(ones(size(vek_rc)));
    leftNaNBorder =data.head.vmax.*(ones(size(vek_rc)));

    % check, where curvature radius > grid
    idx_CurvAreaRight=min_max_v > data.head.vmin  & idx_right;
    idx_CurvAreaLeft =min_max_v < data.head.vmax  & idx_left;

    % create curvature radius border array
    rightNaNBorder(idx_CurvAreaRight) = min_max_v(idx_CurvAreaRight);
    leftNaNBorder(idx_CurvAreaLeft)   = min_max_v(idx_CurvAreaLeft);

    % get z at border
    zleft  = crg_eval_uv2z(data,[uges',(leftNaNBorder +uinc)']);
    zright = crg_eval_uv2z(data,[uges',(rightNaNBorder-uinc)']);

    % check if z isnan
    if sum(isnan(zleft))==size(data.z,1) && sum(isnan(zright))==size(data.z,1) 
        warning('local curvature check succeeded - critical curvature areas in NaN regions')
        ierr = ierr - icerr;
    else
        warning('local curvature check failed - critical curvature areas in z-value regions')
        ierr = ierr + 1;
        % find indices
        iLeft = find(~isnan(zleft));
        iRight = find(~isnan(zright));
        if ~isempty(iLeft)
            idxArr = [iLeft(1) iLeft(end)];
        end
        if ~isempty(iRight)
            idxArr = [iRight(1) iRight(end)];
        end
    end
    
    % remove temp ok
    data = rmfield(data, 'ok');
end

end