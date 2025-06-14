function [data] = crg_read(file)
% CRG_READ Read OpenCRG file.
%   DATA = CRG_READ(FILE) reads OpenCRG (curved regular grid) road data file
%
%   Inputs:
%   FILE    is the OpenCRG file to read
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Example:
%   data = crg_read(file) reads an OpenCRG file.
%
%   See also CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_read.m 
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

%% create data struct array

data = struct;

%% read OpenCRG file as IPLOS data file

ipl = ipl_read(file);

%% look for OpenCRG options

[opts, s] = sdf_cut(ipl.struct, 'ROAD_CRG_OPTS');
if size(s, 2) < size(ipl.struct, 2)
    data.opts = struct; % found CRG options
    ipl.struct = s;
end

%% look for OpenCRG modifiers

[mods, s] = sdf_cut(ipl.struct, 'ROAD_CRG_MODS');
if size(s, 2) < size(ipl.struct, 2)
    data.mods = struct; % found CRG modifiers
    ipl.struct = s;
end

%% look for OpenCRG file reference

[fref s] = sdf_cut(ipl.struct, 'ROAD_CRG_FILE');
if size(s, 2) < size(ipl.struct, 2)
    data.opts = struct; % found OpenCRG file reference
    ipl.struct = s;
    for i = 1:length(fref)
        if strncmp(fref{i}, '*', 1)
            fref{i} = ''; % clear comment lines
        else
            fref{i} = regexprep(fref{i}, '!+.*', ''); % erase inline comments
        end
    end
    fref = strtrim(fref); % remove all leading and trailing blanks
    fref = strcat(fref{1:end}); % concatenate file name
    fenv = regexp(fref, '(?<=\$)\w+', 'match'); % find all potential env vars
    for i = 1:length(fenv)
        fenv{i} = getenv(fenv{i}); % replace env var by its contents
    end
    file = regexprep(fref, '\$\w+', fenv, 'once'); % replace env vars in file name

    ipl = ipl_read(file); % read referenced OpenCRG file
    if ~isfield(data, 'opts') % look here for options only if not already defined
        [opts, s] = sdf_cut(ipl.struct, 'ROAD_CRG_OPTS');
        if size(s, 2) < size(ipl.struct, 2)
            data.opts = struct; % found OpenCRG options
            ipl.struct = s;
        end
    end
    if ~isfield(data, 'mods') % look here for modifiers only if not already defined
        [mods, s] = sdf_cut(ipl.struct, 'ROAD_CRG_MODS');
        if size(s, 2) < size(ipl.struct, 2)
            data.mods = struct; % found CRG modifiers
            ipl.struct = s;
        end
    end
end

%% copy filename to data

data.filenm = ipl.filenm;

%% first OpenCRG data check ...

if size(ipl.kd_dat,1) < 2
    error('at least 2 cross sections required in CRG file %s', file)
end

%% copy comments from ipl.struct to data

[data.ct, ipl.struct] = sdf_cut(ipl.struct, 'CT');

%% read OpenCRG options from ipl.struct (already extracted to opts)

if isfield(data, 'opts')
    for i = 1:length(opts)
        hc = opts{i};
        if strncmp(hc, '*', 1), continue, end % skip comment lines
        hc = regexprep(hc, '!+.*', ''); % erase inline comments

        % all unknown keywords and many syntax flaws will be silently ignored
        [oname, ovalue] = strread(hc, '%s%f', 1, 'delimiter', '=');
        switch lower(deblank(oname{1}))
%         OpenCRG border modes in u and v directions
            case 'border_mode_u'
                data.opts.bdmu = ovalue;
            case 'border_mode_v'
                data.opts.bdmv = ovalue;
            case 'border_offset_u'
                data.opts.bdou = ovalue;
            case 'border_offset_v'
                data.opts.bdov = ovalue;
            case 'border_smooth_ubeg'
                data.opts.bdss = ovalue;
            case 'border_smooth_uend'
                data.opts.bdse = ovalue;
%         OpenCRG reference line continuation
            case 'refline_continuation'
                data.opts.rflc = ovalue;
%         OpenCRG reference line search strategy
            case 'refline_search_far'
                data.opts.sfar = ovalue;
            case 'refline_search_close'
                data.opts.scls = ovalue;
%         OpenCRG message options
            case 'warn_msgs'
                data.opts.wmsg = ovalue;
            case 'warn_curv_local'
                data.opts.wcvl = ovalue;
            case 'log_msgs'
                data.opts.lmsg = ovalue;
            case 'log_eval'
                data.opts.leva = ovalue;
            case 'log_eval_freq'
                data.opts.levf = ovalue;
            case 'log_hist'
                data.opts.lhst = ovalue;
            case 'log_hist_freq'
                data.opts.lhsf = ovalue;
            case 'log_stat'
                data.opts.lsta = ovalue;
            case 'log_stat_freq'
                data.opts.lstf = ovalue;
%         OpenCRG check options
            case 'check_eps'
                data.opts.ceps = ovalue;
            case 'check_inc'
                data.opts.cinc = ovalue;
            case 'check_tol'
                data.opts.ctol = ovalue;
            case 'check_pro'
                data.opts.cpro = ovalue;
            case 'check_wgs'
                data.opts.cwgs = ovalue;
        end
    end
end

data = crg_check_opts(data);

%% read OpenCRG modifiers from ipl.struct (already extracted to mods)

if isfield(data, 'mods')
    for i = 1:length(mods)
        hc = mods{i};
        if strncmp(hc, '*', 1), continue, end % skip comment lines
        hc = regexprep(hc, '!+.*', ''); % erase inline comments

        % all unknown keywords and many syntax flaws will be silently ignored
        [mname, mvalue] = strread(hc, '%s%f', 1, 'delimiter', '=');
        switch lower(deblank(mname{1}))
%         OpenCRG scaling
            case 'scale_z_grid'
                data.mods.szgd = mvalue;
            case 'scale_slope'
                data.mods.sslp = mvalue;
            case 'scale_banking'
                data.mods.sbkg = mvalue;
            case 'scale_length'
                data.mods.slth = mvalue;
            case 'scale_width'
                data.mods.swth = mvalue;
            case 'scale_curvature'
                data.mods.scrv = mvalue;
%         OpenCRG elevation grid NaN handling
            case 'grid_nan_mode'
                data.mods.gnan = mvalue;
            case 'grid_nan_offset'
                data.mods.gnao = mvalue;
%         OpenCRG re-positioning: refline by offset
            case 'refline_rotcenter_x'
                data.mods.rlrx = mvalue;
            case 'refline_rotcenter_y'
                data.mods.rlry = mvalue;
            case 'refline_offset_phi'
                data.mods.rlop = mvalue;
            case 'refline_offset_x'
                data.mods.rlox = mvalue;
            case 'refline_offset_y'
                data.mods.rloy = mvalue;
            case 'refline_offset_z'
                data.mods.rloz = mvalue;
%         OpenCRG re-positioning: refline by refpoint
            case 'refpoint_u'
                data.mods.rptu = mvalue;
            case 'refpoint_u_fraction'
                data.mods.rpfu = mvalue;
            case 'refpoint_u_offset'
                data.mods.rpou = mvalue;
            case 'refpoint_v'
                data.mods.rptv = mvalue;
            case 'refpoint_v_fraction'
                data.mods.rpfv = mvalue;
            case 'refpoint_v_offset'
                data.mods.rpov = mvalue;
            case 'refpoint_x'
                data.mods.rptx = mvalue;
            case 'refpoint_y'
                data.mods.rpty = mvalue;
            case 'refpoint_z'
                data.mods.rptz = mvalue;
            case 'refpoint_phi'
                data.mods.rptp = mvalue;
        end
    end
end

data = crg_check_mods(data);

%% read OpenCRG header information from ipl.struct

[block, ipl.struct] = sdf_cut(ipl.struct, 'ROAD_CRG');

data.head = struct;
for i = 1:length(block)
    hc = block{i};
    if strncmp(hc, '*', 1), continue, end % skip comment lines
    hc = regexprep(hc, '!+.*', ''); % erase inline comments

    % all unknown keywords and many syntax flaws will be silently ignored
    [hname, hvalue] = strread(hc, '%s%f', 1, 'delimiter', '=');
    switch lower(deblank(hname{1}))
        case 'reference_line_start_u'
            data.head.ubeg = hvalue;
        case 'reference_line_end_u'
            data.head.uend = hvalue;
        case 'reference_line_increment'
            data.head.uinc = hvalue;
        case 'long_section_v_right'
            data.head.vmin = hvalue;
        case 'long_section_v_left'
            data.head.vmax = hvalue;
        case 'long_section_v_increment'
            data.head.vinc = hvalue;
        case 'reference_line_start_b'
            data.head.bbeg = hvalue;
        case 'reference_line_end_b'
            data.head.bend = hvalue;
        case 'reference_line_start_s'
            data.head.sbeg = hvalue;
        case 'reference_line_end_s'
            data.head.send = hvalue;
%
        case 'reference_line_start_x'
            data.head.xbeg = hvalue;
        case 'reference_line_start_y'
            data.head.ybeg = hvalue;
        case 'reference_line_start_z'
            data.head.zbeg = hvalue;
        case 'reference_line_start_phi'
            data.head.pbeg = hvalue;
        case 'reference_line_end_x'
            data.head.xend = hvalue;
        case 'reference_line_end_y'
            data.head.yend = hvalue;
        case 'reference_line_end_z'
            data.head.zend = hvalue;
        case 'reference_line_end_phi'
            data.head.pend = hvalue;
        case 'reference_line_offset_x'
            data.head.xoff = hvalue;
        case 'reference_line_offset_y'
            data.head.yoff = hvalue;
        case 'reference_line_offset_z'
            data.head.zoff = hvalue;
        case 'reference_line_offset_phi'
            data.head.poff = hvalue;
%
        case 'reference_line_start_lon'
            data.head.ebeg = hvalue;
        case 'reference_line_start_lat'
            data.head.nbeg = hvalue;
        case 'reference_line_start_alt'
            data.head.abeg = hvalue;
        case 'reference_line_end_lon'
            data.head.eend = hvalue;
        case 'reference_line_end_lat'
            data.head.nend = hvalue;
        case 'reference_line_end_alt'
            data.head.aend = hvalue;
    end
end

data = crg_check_head(data);

%% read OpenCRG map projection information from ipl.struct

[block, ipl.struct] = sdf_cut(ipl.struct, 'ROAD_CRG_MPRO');

if length(block) > 0
    data.mpro = struct;
    
    data.mpro.gell = struct;
    data.mpro.lell = struct;
    data.mpro.tran = struct;
    data.mpro.proj = struct;
    
    for i = 1:length(block)
        hc = block{i};
        if strncmp(hc, '*', 1), continue, end % skip comment lines
        hc = regexprep(hc, '!+.*', ''); % erase inline comments
        
        % all unknown keywords and many syntax flaws will be silently ignored
        [pname, pvalue] = strread(hc, '%s%s', 1, 'delimiter', '=');
        pvalue = pvalue{1};
        switch lower(deblank(pname{1}))
            case 'gell_nm'
                data.mpro.gell.nm = regexp(pvalue, '(?<=\'')\w+', 'match', 'once');
            case 'gell_a'
                data.mpro.gell.a = str2double(pvalue);
            case 'gell_b'
                data.mpro.gell.b = str2double(pvalue);
                %
            case 'tran_nm'
                data.mpro.tran.nm = regexp(pvalue, '(?<=\'')\w+', 'match', 'once');
            case 'tran_ds'
                data.mpro.tran.ds = str2double(pvalue);
            case 'tran_rx'
                data.mpro.tran.rx = str2double(pvalue);
            case 'tran_ry'
                data.mpro.tran.ry = str2double(pvalue);
            case 'tran_rz'
                data.mpro.tran.rz = str2double(pvalue);
            case 'tran_tx'
                data.mpro.tran.tx = str2double(pvalue);
            case 'tran_ty'
                data.mpro.tran.ty = str2double(pvalue);
            case 'tran_tz'
                data.mpro.tran.tz = str2double(pvalue);
                %
            case 'lell_nm'
                data.mpro.lell.nm = regexp(pvalue, '(?<=\'')\w+', 'match', 'once');
            case 'lell_a'
                data.mpro.lell.a = str2double(pvalue);
            case 'lell_b'
                data.mpro.lell.b = str2double(pvalue);
                %
            case 'proj_nm'
                data.mpro.proj.nm = regexp(pvalue, '(?<=\'')\w+', 'match', 'once');
            case 'proj_f0'
                data.mpro.proj.f0 = str2double(pvalue);
            case 'proj_p0'
                data.mpro.proj.p0 = str2double(pvalue);
            case 'proj_l0'
                data.mpro.proj.l0 = str2double(pvalue);
            case 'proj_e0'
                data.mpro.proj.e0 = str2double(pvalue);
            case 'proj_n0'
                data.mpro.proj.n0 = str2double(pvalue);
        end
    end
    
    data = crg_check_mpro(data);
end

%% copy extra struct data if available

if length(ipl.struct) > 0 %#ok<ISMT>
    data.struct = ipl.struct;
end

%% evaluate virtual U channel definitions in ipl.kd_ind

uinc = 0.010; % default value
if isfield(data.head, 'uinc'), uinc = data.head.uinc; end

ubeg = 0.000; % default value
if isfield(data.head, 'ubeg'), ubeg = data.head.ubeg; end

for i=1:length(ipl.kd_ind)
    hc = ipl.kd_ind{i};
    [ht, hc, hb, hi] = strread(hc, '%s%s%f%f', 1, 'delimiter', ',', 'emptyvalue', NaN);
    ht = deblank(strjust(ht{1}, 'left')); %#ok<TRIM2>
    hc = deblank(strjust(hc{1}, 'left')); %#ok<TRIM2>
    if strcmp(ht, 'reference line u')
        if strcmp(hc, 'm')
            if length(hb) == 1
                if ~isnan(hb), ubeg = hb; end
            end
            if length(hi) == 1
                if ~isnan(hi), uinc = hi; end
            end
        end
    end
end

uend = ubeg + (size(ipl.kd_dat,1)-1)*uinc;

if ubeg == 0.000
    data.u = uend;
else
    data.u = [ubeg uend];
end

%% evaluate dependant D channel definitions in ipl.kd_def

ichanp = 0;
ichans = 0;
ichanb = 0;
nchanv = 0;
nposv  = 0;
nnumv  = 0;
dchanv = [];
ichanv = [];
for i=1:length(ipl.kd_def)
    hc = ipl.kd_def{i};
    [ht, hc] = strread(hc, '%s%s', 1, 'delimiter', ',');
    ht = deblank(strjust(ht{1}, 'left')); %#ok<TRIM2>
    hc = deblank(strjust(hc{1}, 'left')); %#ok<TRIM2>
    if strcmp(ht, 'reference line phi')
        if strcmp(hc, 'rad')
            ichanp = i;
        else
            error('reference line phi unit error in CRG file %s', file)
        end
    elseif strcmp(ht, 'reference line slope')
        if strcmp(hc, 'm/m')
            ichans = i;
        else
            error('reference line slope unit error in CRG file %s', file)
        end
    elseif strcmp(ht, 'reference line banking')
        if strcmp(hc, 'm/m')
            ichanb = i;
        else
            error('reference line banking unit error in CRG file %s', file)
        end
    elseif strncmp(ht, 'long section ', 13)
        if strncmp(ht(14:end), 'at v = ', 7) % long section defined by position
            nposv = nposv + 1;
            hd = str2double(ht(21:end));
        else % long section defined by number
            nnumv = nnumv + 1;
            hd = str2double(ht);
        end
        if strcmp(hc, 'm') % register only with valid unit, report errors below
            nchanv = nchanv + 1;
            dchanv(nchanv) = hd; %#ok<AGROW>
            ichanv(nchanv) = i; %#ok<AGROW>
        end
    end
end
if (nposv > 0) && (nnumv > 0)
    error('inconsistent long section definition in CRG file %s', file)
end
if nchanv ~= (nposv+nnumv)
    error('long section unit definition error in CRG file %s', file)
end
if nchanv < 2
    error('at least 2 long sections required in CRG file %s', file)
end

%% evaluate long section spacing

[dchanv, ix] = sort(dchanv);
ichanv = ichanv(ix);

if nposv > 0  % long sections defined by positions
    data.v = dchanv;
elseif nnumv > 0 % long sections defined by numbers
    if max(abs(dchanv - [1:nnumv])) > 0.001 %#ok<NBRAK> % numbers must be [1:nnumv]
        error('illegal long section position number(s) in CRG file %s', file);
    end

    vinc = 0.010; % default
    if isfield(data.head, 'vinc'), vinc = data.head.vinc; end

    vmin = -vinc*(nchanv-1)/2; % default
    if isfield(data.head, 'vmin'), vmin = data.head.vmin; end

    data.v = vmin + [0:nnumv-1]*vinc; %#ok<NBRAK>
end

%% set reference line heading

if ichanp ~= 0
    data.p = ipl.kd_dat(2:end,ichanp)'; % ignore first value of ipl.kd_dat
elseif isfield(data.head, 'pbeg')
    data.p = data.head.pbeg;
end

%% set slope

if ichans ~= 0
    data.s = ipl.kd_dat(2:end,ichans)'; % ignore first value of ipl.kd_dat
elseif isfield(data.head, 'sbeg')
    data.s = data.head.sbeg;
end

%% set cross slope

if ichanb ~= 0
    data.b = ipl.kd_dat(:,ichanb)';
elseif isfield(data.head, 'bbeg')
    data.b = data.head.bbeg;
end

%% set z array from ipl.kd_dat

data.z(:,:) = ipl.kd_dat(:,ichanv);

%% force single

data = crg_single(data);

%% check data consistency

data = crg_check(data);

end
