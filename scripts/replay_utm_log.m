%% Program to animate UTM logs

clear variables ;
close all ;
sctrs = 6;
domaindir = 'build/Domains/' ;
trackerdir = 'build/Tracker/' ;
sctrs = 11;
domaindir = ['Domains' filesep];
trackerdir = ['Tracker' filesep];
mode = ''; % can change this to look at a different folder
run = 0 ;
epoch = 0 ;
eval = 0 ;

% Filenames
nodesFile = [domaindir num2str(sctrs) '_Sectors' mode '/nodes.csv'] ; % node locations
edgesFile = [domaindir num2str(sctrs) '_Sectors' mode '/edges.csv'] ; % edge connections
capsFile = [trackerdir num2str(sctrs) '_Sectors' mode '/capacities.csv'] ; % link capacities
statesFile = [trackerdir num2str(sctrs) '_Sectors' mode '/states_run' num2str(run) '_epoch' num2str(epoch) '_eval' num2str(eval) '.csv'] ; % logged agent states
weightsFile = [trackerdir num2str(sctrs) '_Sectors' mode '/weights_run' num2str(run) '_epoch' num2str(epoch) '_eval' num2str(eval) '.csv'] ; % logged agent output costs
uavLogFile = [trackerdir num2str(sctrs) '_Sectors' mode '/uav_pos_run' num2str(run) '_epoch' num2str(epoch) '_eval' num2str(eval) '.csv'] ; % logged UAV traffic states
uavEndSectFile = [trackerdir num2str(sctrs) '_Sectors' mode '/uav_end_sectors_run' num2str(run) '_epoch' num2str(epoch) '_eval' num2str(eval) '.csv'] ;
uavWaitFile = [trackerdir num2str(sctrs) '_Sectors' mode '/uav_wait_run' num2str(run) '_epoch' num2str(epoch) '_eval' num2str(eval) '.csv'] ;
edgeTimeFile = [trackerdir num2str(sctrs) '_Sectors' mode '/edgeTime.csv'] ; % time required to cross each link

% Toggle
pUAV = 1 ;
pUAVnums = 1;
pUAVwait = 0;
pCap = 1 ;
pCost = 1 ;
pEnd = 0 ;

%% Load graph parameters

nodes = csvread(nodesFile) ;
edges = csvread(edgesFile) ;
caps = csvread(capsFile) ;
eInd = edges + 1 ;
% edgeTime = csvread(edgeTimeFile) ; % toggle if edgeTimeFile available
edgeTime = zeros(size(eInd,1),1) ; % toggle if edgeTimeFile available
edgeLen = zeros(size(eInd,1),1) ;
xrange = max(nodes(:,1)) - min(nodes(:,1)) ;
yrange = max(nodes(:,2)) - min(nodes(:,2)) ;
d = 0.01 ; % separate links in plot
cbar = 0.2 ; % per uav on capacity bar
fs = 12 ; % font size
dscale = 2 ; % time and distance scaling

% Plot graph
% figure('units','normalized','outerposition',[0 0 1 1])
figure(1)
hold on ;
axis equal ;
alpha = zeros(size(eInd,1),1) ;
hN = plot(nodes(:,1),nodes(:,2),'ko','markersize',15,'linewidth',3) ;
for i = 1:size(nodes, 1)
    text(nodes(i,1)-0.04, nodes(i,2), num2str(i-1),'fontsize',8,'color',[0.5 0.5 0.5]) ;
end
hE = zeros(size(eInd,1),1) ;
hBu = zeros(size(eInd,1),1) ;
hB = zeros(size(eInd,1),1) ;
hT = zeros(size(eInd,1),1) ;
hS = cell(size(eInd,1),1) ;
hStext = cell(size(eInd,1),1) ;
for i = 1:size(eInd,1)
    x = [nodes(eInd(i,1),1),nodes(eInd(i,2),1)] ;
    y = [nodes(eInd(i,1),2),nodes(eInd(i,2),2)] ;
    diffx = x(2) - x(1) ;
    diffy = y(2) - y(1) ;
    edgeLen(i) = sqrt(diffx^2 + diffy^2) ;
%     edgeTime(i) = dscale*ceil(edgeLen(i)) ; % toggle if edgeTimeFile available
    % blue paths map traffic going to the right, ties broken by paths going
    % upwards. Red paths otherwise.
    alpha(i) = atan2(diffy,diffx) ;
    dx = -d*xrange*sin(alpha(i)) ;
    dy = d*yrange*cos(alpha(i)) ;
    if (diffx > 0 || (diffx == 0 && diffy > 0) )
        c = [0.5,0.5,1] ; % light blue
        cc = [0.75,0.75,1] ;
        cb = [0.5,0.5,1] ;
    else
        c = [1,0.5,0.5] ; % light red
        cc = [1,0.75,0.75] ;
        cb = [1,0.25,0.25] ;
    end
    hE(i) = plot(x+dx,y+dy,'-','color',c,'linewidth',2) ;
    
    % Plot capacity bars
    hBscale = 2 ;
    hBthick = 12 ;
    midx = diffx/2 + x(1) ;
    midy = diffy/2 + y(1) ;
    barx(1) = midx - cbar*caps(i)/2*cos(alpha(i)) ;
    barx(2) = midx + cbar*caps(i)/2*cos(alpha(i)) ;
    bary(1) = midy - cbar*caps(i)/2*sin(alpha(i)) ;
    bary(2) = midy + cbar*caps(i)/2*sin(alpha(i)) ;
    if pCap
        hBu(i) = plot(barx+hBscale*dx,bary+hBscale*dy,'-','color',cc,'linewidth',hBthick) ;
        hB(i) = plot(barx(1)+hBscale*dx,bary(1)+hBscale*dy,'-','color',cb,'linewidth',hBthick) ;
    end
    
    % Plot agent output costs
    hTscale = 2 ;
    if pCost
        hT(i) = text(barx(1)+hTscale*dx,bary(1)+hTscale*dy,'a:','rotation',alpha(i)*180/pi,'fontsize',fs) ;
    end
    
    % Plot UAV motion
    if pUAV
        s = 2*caps(i) + 2 ;
        xs = linspace(x(1),x(2),s) + dx ;
        ys = linspace(y(1),y(2),s) + dy ;
        sInd = 3:2:numel(xs)-1 ;
        hS{i} = zeros(numel(sInd),50) ;
        hStext{i} = zeros(numel(sInd),50) ; % text objects for writing number of uav
        for j = 1:numel(sInd)
            for k = 1:50
                hS{i}(k) = plot(xs(sInd(j)),ys(sInd(j)),'o','color','none','markersize',10,'linewidth',2) ;
                hStext{i}(k) = text(xs(sInd(j)),ys(sInd(j)),'','FontSize',fs/(4/3),'FontWeight','bold') ;
%             hS{i}(j) = plot(xs(sInd(j)),ys(sInd(j)),'o','color',cc,'markersize',10,'linewidth',2) ;
            end
        end
    end
end
axis tight ;

%% Load log files
states = csvread(statesFile) ;
weights = csvread(weightsFile) ;
tFinal = size(states,1) ;
nAgents = size(edges,1) ;
team = 1:nAgents:size(states,2) ;

% Trim extra columns
if (mod(size(states,2),nAgents) ~= 0)
    team = team(1:end-1) ;
    fprintf('Ignoring extra columns %i:%i\n',team(end)+1,size(states,2)) ;
end
fprintf('Displaying %is episode logs from %i UTM agent teams\n',tFinal,numel(team)) ;

%% Annotate graph
figure(1)
pause ;

% How many uavs were traveling/delayed in last time step
lastUAVcount = 0;

if pUAV
    fid = fopen(uavEndSectFile) ;
    usCell = textscan(fid,'%s','delimiter','\n');
    fclose(fid);
    uavEndSectors = usCell{1} ;
    
    fid = fopen(uavWaitFile) ;
    uwCell = textscan(fid, '%s', 'delimiter', '\n');
    fclose(fid);
    uavWait = uwCell{1} ;
    
    fid = fopen(uavLogFile);
    uCell = textscan(fid,'%s','delimiter','\n') ;
    fclose(fid);
    uavStates = uCell{1} ;
    uCheck = csvread(uavLogFile) ;
    uMax = zeros(numel(team),1) ;
    j = 1 ;
    for i = 1:tFinal:size(uCheck,1)
        uC = uCheck(i:i+tFinal-1,1:2:end) ;
        uMax(j) = max(max(uC)) + 1 ; % maximum uav ID in episode
    end
end

for utm = 1:numel(team)
    if pUAV
        hUAVs = -ones(uMax(utm),1) ; % record UAV plot handles, default -1
        hUAVnum = -ones(uMax(utm),1) ; % text handles
        hUAVwait = -ones(size(nodes, 1));
    end
    for t = 1:tFinal
        title(sprintf('UTM Team %i, t=%is',utm,t),'fontsize',fs) ;
        for i = 1:size(eInd,1)
            nUAVs = states(t,team(utm)+i-1) ;
            if nUAVs > caps(i)
%                 fprintf('DATA ERROR: State [nUAVs=%i] exceeds link agent %i''s capacity!\n',nUAVs,i-1) ;
%                 fprintf('Thresholding value for plotting purposes.\n') ;
                nUAVs = caps(i) ;
            end
            
            % Plot capacity bars
            if pCap
                eBarX = get(hBu(i),'xdata') ;
                eBarY = get(hBu(i),'ydata') ;
                if (nUAVs ~= caps(i))
                    uavFill = nUAVs/(caps(i)+1) ;
                else
                    uavFill = nUAVs/caps(i) ;
                end
                eFillX(1) = eBarX(1) ;
                eFillY(1) = eBarY(1) ;
                eFillX(2) = uavFill * (eBarX(2)-eBarX(1)) + eFillX(1) ;
                eFillY(2) = uavFill * (eBarY(2)-eBarY(1)) + eFillY(1) ;
                set(hB(i),'xdata',eFillX,'ydata',eFillY) ;
            end
            
            % Plot agent output costs
            if pCost
                set(hT(i),'string',sprintf('a: %.1f',weights(t,team(utm)+i-1))) ;
            end
        end
           
        uavPosMap = containers.Map();
        
        % Plot UAVs
        if pUAV
            usData = str2num(uavEndSectors{t+(utm-1)*tFinal}) ;
            usMat = zeros(numel(usData)/2,2) ;
            usMat(:,1) = usData(1:2:end)' ;
            usMat(:,2) = usData(2:2:end)' ;
            usMat = usMat + 1 ;
            
            uwData = str2num(uavWait{t+(utm-1)*tFinal}) ;
            uwMat = zeros(numel(uwData)/2,2) ;
            uwMat(:,1) = uwData(1:2:end)' ;
            uwMat(:,2) = uwData(2:2:end)' ;
            uwMat = uwMat + 1 ;
            
            uData = str2num(uavStates{t+(utm-1)*tFinal}) ;
            uMat = zeros(numel(uData)/2,2) ;
            uMat(:,1) = uData(1:2:end)' ;
            uMat(:,2) = uData(2:2:end)' ;
            uMat = uMat + 1 ;
            
            if t == 1
                uMat0 = [] ;
            end
            
            for ii = 1:size(uMat0,1) % look for change from prior state, esp. find UAVs that have been removed
                bGoal = true ;
                uRem = false(size(uMat,1),1) ;
                if ~isnan(uMat(1))
                    for jj = 1:size(uMat,1)
                        if uMat0(ii,1) == uMat(jj,1)
                            bGoal = false ; % UAV has not reached goal
                            if uMat0(ii,2) == uMat(jj,2) % check if UAV is on the same link
                                if size(offsetMult, 1) < uMat(jj,1)
                                    offsetMult(uMat(jj,1)) = 0;
                                end
                                offs = offsetMult(uMat(jj,1)) + 1;
                                xx = get(hUAVs(uMat(jj,1)),'xdata') ;
                                yy = get(hUAVs(uMat(jj,1)),'ydata') ;
                                dx = min(1,edgeLen(uMat(jj,2))/edgeTime(uMat(jj,2)))*cos(alpha(uMat(jj,2))) ;
                                dy = min(1,edgeLen(uMat(jj,2))/edgeTime(uMat(jj,2)))*sin(alpha(uMat(jj,2))) ;
                                xe = get(hE(uMat(jj,2)),'xdata') ;
                                ye = get(hE(uMat(jj,2)),'ydata') ;
                                xu = min(max(xx+dx,min(xe)),max(xe)) ;
                                yu = min(max(yy+dy,min(ye)),max(ye)) ;

                                s = [num2str(xu) ' ' num2str(yu)] ;
                                if ~ismember(s, keys(uavPosMap))
                                    uavPosMap(s) = [] ;
                                end

                                uavPosMap(s) = [uavPosMap(s) uMat(jj,1)];

                                set(hUAVs(uMat(jj,1)),'xdata',xu,'ydata',yu) ; % move along edge
    %                             set(hUAVnum(uMat(jj,1)),'Position', [xu+0.1*offs, yu],'String',num2str(uMat(jj,1)-1)) ; % move along edge
                            else % UAV has moved onto another link
                                uec = get(hUAVs(uMat0(ii,1)),'color') ;
                                ufc = get(hUAVs(uMat0(ii,1)),'markerfacecolor') ;
                                set(hUAVs(uMat0(ii,1)),'color','none','markerfacecolor','none') ;
    %                             set(hUAVnum(uMat0(ii,1)),'String','') ; % erase number
                                bEmpty = false ; % search for empty space
                                for kk = 1:numel(hS{uMat(jj,2)})
                                    if strcmp(get(hS{uMat(jj,2)}(kk),'color'),'none')
                                        if size(offsetMult, 1) < uMat(jj,1)
                                            offsetMult(uMat(jj,1)) = 0;
                                        end
                                        offs = offsetMult(uMat(jj,1)) + 1;
                                        bEmpty = true ;
                                        hUAVs(uMat(jj,1)) = hS{uMat(jj,2)}(kk) ;
    %                                     hUAVnum(uMat(jj,1)) = hStext{uMat(jj,2)}(kk) ;
                                        xe = get(hE(uMat(jj,2)),'xdata') ;
                                        ye = get(hE(uMat(jj,2)),'ydata') ;
                                        xu = edgeLen(uMat(jj,2))/edgeTime(uMat(jj,2))*cos(alpha(uMat(jj,2)))+xe(1) ;
                                        yu = edgeLen(uMat(jj,2))/edgeTime(uMat(jj,2))*sin(alpha(uMat(jj,2)))+ye(1) ;

                                        s = [num2str(xu) ' ' num2str(yu)] ;
                                        if ~ismember(s, keys(uavPosMap))
                                            uavPosMap(s) = [] ;
                                        end

                                        uavPosMap(s) = [uavPosMap(s) uMat(jj,1)];

                                        set(hUAVs(uMat(jj,1)),'xdata',xu,'ydata',yu,'color',uec,'markerfacecolor',ufc) ;
    %                                     set(hUAVnum(uMat(jj,1)),'Position',[xu+0.1*offs, yu],'String',num2str(uMat(jj,1)-1)) ;
                                        break ;
                                    end
                                end
                                if ~bEmpty % link agent capacity breached
                                    fprintf('DATA ERROR: UAV %i violating link agent %i''s capacity! (%i -> %i)\n',uMat(jj,1)-1,uMat(jj,2)-1, eInd(uMat(jj,2), 1), eInd(uMat(jj,2), 2)) ;
    %                                 fprintf('Removing UAV %i from rollover state\n',uMat(jj,1)-1) ;
    %                                 uRem(jj) = true ;
                                end
                            end
                        end
                    end
                end
                uMat(uRem,:) = [] ;
                if bGoal && ~isnan(uMat0(1))% UAV has reached goal and has been removed
                    set(hUAVs(uMat0(ii,1)),'color','none','markerfacecolor','none') ;
%                     set(hUAVnum(uMat0(ii,1)),'String','') ;
                    hUAVs(uMat0(ii,1)) = -1 ; % return to default value
%                     hUAVnum(uMat0(ii,1)) = -1 ; % return to default value
                end
            end
            uRem = false(size(uMat,1),1) ;
            linkUavCount = zeros(size(caps));
            if ~isnan(uMat(1))
                for ii = 1:size(uMat,1) % look for new UAVs
                    linkUavCount(uMat(ii,2)) = linkUavCount(uMat(ii,2)) + 1;
                    bNew = true ;
                    for jj = 1:size(uMat0,1)
                        if uMat(ii,1) == uMat0(jj,1)
                            bNew = false ; % not a new UAV
                            break ;
                        end
                    end
                    if bNew % new UAV in system
                        uec = 'k' ;
                        ufc = rand(1,3) ; % assign random colour
                        bEmpty = false ; % search for empty space
                        for kk = 1:numel(hS{uMat(ii,2)})
                            if strcmp(get(hS{uMat(ii,2)}(kk),'color'),'none')
                                if exist('offsetMult', 'var')
                                    if size(offsetMult, 1) < uMat(ii,1)
                                        offsetMult(uMat(ii,1)) = 0;
                                    end
                                else
                                    offsetMult(uMat(ii,1)) = 0;
                                end

                                if linkUavCount(uMat(ii,2)) <= caps(uMat(ii,2))
                                    bEmpty = true ;
                                else
                                    offsetMult(uMat(ii,1)) = mod(offsetMult(uMat(ii,1)), 4) + 1;
                                end
                                offs = offsetMult(uMat(ii,1)) + 1;
                                xe = get(hE(uMat(ii,2)),'xdata') ;
                                ye = get(hE(uMat(ii,2)),'ydata') ;
                                hUAVs(uMat(ii,1)) = hS{uMat(ii,2)}(kk) ;
    %                             hUAVnum(uMat(ii,1)) = hStext{uMat(ii,2)}(kk) ;
                                xu = edgeLen(uMat(ii,2))/edgeTime(uMat(ii,2))*cos(alpha(uMat(ii,2)))+xe(1) ;
                                yu = edgeLen(uMat(ii,2))/edgeTime(uMat(ii,2))*sin(alpha(uMat(ii,2)))+ye(1) ;
                                set(hUAVs(uMat(ii,1)),'xdata',xu,'ydata',yu,'color',uec,'markerfacecolor',ufc) ;

                                s = [num2str(xu) ' ' num2str(yu)] ;
                                if ~ismember(s, keys(uavPosMap))
                                    uavPosMap(s) = [] ;
                                end

                                uavPosMap(s) = [uavPosMap(s) uMat(ii,1)];

    %                             set(hUAVnum(uMat(ii,1)),'Position',[xu+0.1*offs, yu],'String',num2str(uMat(ii,1)-1)) ;
                                if pEnd
                                    fprintf('UAV %i created. Heading to Sector %i\n', usMat(ii,1)-1, usMat(ii,2)-1) ;
                                end
    %                            text(xu + 0.1, yu, num2str(uMat(ii,1)), 'FontSize', fs, 'FontWeight', 'bold') ;
                                break ;
                            end
                        end
                        if ~bEmpty % link agent capacity breached
                            fprintf('DATA ERROR: UAV %i violating link agent %i''s capacity! (%i -> %i)\n',uMat(ii,1)-1,uMat(ii,2)-1, eInd(uMat(ii,2), 1), eInd(uMat(ii,2), 2)) ;
    %                         fprintf('Removing UAV %i from rollover state\n',uMat(ii,1)-1) ;
    %                         uRem(ii) = true ;
                        end
                    end
                end
            end
            uMat(uRem,:) = [] ;
            uMat0 = uMat ;
        end
        
        if pUAVnums
            % Draw numbers for UAVs
            
            % Delete existing numbers (recreate them each time to save
            %   debugging frustration :D )
            % TODO: Make this more elegant
            for i = 1:lastUAVcount
                if hUAVnum(i) ~= -1
                    delete(hUAVnum(i));
                    hUAVnum(i) = -1;
                end
            end
            
            % Keeps track of how many UAVs are traveling/delayed
            count = 0;
            % Go through keys. Keys are xy positions of UAVs
            for strPos = keys(uavPosMap)
                s = strPos{1};
                pos = sscanf(s, '%f %f');
                xu = pos(1);
                yu = pos(2);
                % Get values, which are nums of UAVs at position
                uavNums = uavPosMap(s);
                % Go through UAV nums
                for i = 1:numel(uavNums)
                    count = count + 1;
                    % Text position in x
                    tx = xu+0.2*i-0.1;
                    % Text position in y
                    ty = yu;
                    
                    hUAVnum(count) = text(tx, ty, num2str(uavNums(i) - 1), 'FontSize', fs/(4/3), 'FontWeight', 'bold') ;
                end            
            end
            lastUAVcount = count;
        end
        
        if pUAVwait
            % Draw numbers for UAVs
            count = 0;
            uavWaitCount = containers.Map();
            if numel(uwMat) > 0 && ~isnan(uwMat(1))
                for i = 1:numel(uwMat)/2
                    s = num2str(uwMat(i, 2));
                    if ~ismember(s, keys(uavWaitCount))
                        uavWaitCount(s) = 1 ;
                    else
                        uavWaitCount(s) = uavWaitCount(s) + 1;
                    end
                end
                for strNode = keys(uavWaitCount)
                    n = str2num(strNode{1});
                    waitCount = uavWaitCount(strNode{1});
                    xn = nodes(n,1);
                    yn = nodes(n,2);
                    
                    tx = xn+0.1;
                    ty = yn-0.1;
                    if hUAVwait(n) == -1
                        hUAVwait(n) = text(tx, ty, num2str(waitCount), 'FontSize', fs, 'FontWeight', 'bold', 'color', 'r') ;
                    else
                        set(hUAVwait(n),'Position',[tx, ty],'String',num2str(waitCount) );
                    end      
                end
            end
                

            for i = 1:size(nodes,1)
                s = num2str(i);
                if ~ismember(s, keys(uavWaitCount))
                    if hUAVwait(i) ~= -1
                        set(hUAVwait(n),'Position',[tx, ty],'String','' );
                        delete(hUAVwait(i));
                        hUAVwait(i) = -1;
                    end
                    hUAVwait(i) = -1;
                end
            end
        end
        
        drawnow ;
        pause ;
		% For creating frames used to create a video
%        F = getframe(gcf);
%        stop = t*5;
%        start = (t-1)*5+1;
%        for i = start:stop
%            imwrite(F.cdata, ['image' mode num2str(i) '.png']);
%        end
    end
    for i = 1:size(hS,1)
        for j = 1:size(hS{i},1)
            set(hS{i}(j),'color','none','markerfacecolor','none') ;
        end
    end
end
