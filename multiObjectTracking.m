%% Motion-Based Multiple Object Tracking
% This performs automatic detection and motion-based
% tracking of moving objects in a video from a stationary camera.

%%
% The problem of motion-based object tracking can be divided into two parts:
%
% # detecting moving objects in each frame 
% # associating the detections corresponding to the same object over time
%
% The detection of moving objects uses a background subtraction algorithm
% based on Gaussian mixture models. Morphological operations are applied to
% the resulting foreground mask to eliminate noise. Finally, blob analysis
% detects groups of connected pixels, which are likely to correspond to
% moving objects. 
%
% The association of detections to the same object is based solely on
% motion. The motion of each track is estimated by a Kalman filter. The
% filter is used to predict the track's location in each frame, and
% determine the likelihood of each detection being assigned to each 
% track.
%
% Track maintenance becomes an important aspect of this example. In any
% given frame, some detections may be assigned to tracks, while other
% detections and tracks may remain unassigned.The assigned tracks are
% updated using the corresponding detections. The unassigned tracks are 
% marked invisible. An unassigned detection begins a new track. 
%
% Each track keeps count of the number of consecutive frames, where it
% remained unassigned. If the count exceeds a specified threshold, the
% example assumes that the object left the field of view and it deletes the
% track.  
%

%% Multi Object Tracking
% The |multiObjectTracking| function is the top level function.
%
% * |camList| :             An array which contains the IP addresses of all cams
% * |serverURL| :           URL of server which recieves blob information
%
% Only if a connection is established with the server the blob tracking is 
% done, else the program fails to run.

function multiObjectTracking()

%14: observation room
%15: leftmost learning lab
%11: second leftmost learning lab
%10: second rightmost learning lab
%12: rightmost learning lab
camList = ['http://192.168.1.14/video.cgi?.mjpg'];
    

serverURL = 'dev.mirrorworlds.icat.vt.edu';

% Create system objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects();

tracks = initializeTracks(); % Create an empty array of tracks.

nextId = 1; % ID of the next track

% Server TCP/IP connection
serverObj = tcpip(serverURL,9999);
serverObj.OutputBufferSize = 4096;

status = serverObj.status;

%if(strcmp(status, 'closed'))
if(false)
   fprintf('Connection to server NOT established.\nProgram terminated\n');
else
   fopen(serverObj);
   % Detect moving objects, and track them across video frames.
   while (1)
      frame = readFrame();
      [centroids, bboxes, mask] = detectObjects(frame);
      predictNewLocationsOfTracks();
      [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment();
    
      updateAssignedTracks();
      updateUnassignedTracks();
      deleteLostTracks();
      createNewTracks();
      displayTrackingResults();
   end
end

%% Create System Objects
% Create System objects used for reading the video frames, detecting
% foreground objects, and displaying results.

    function obj = setupSystemObjects()
        % Initialize Video I/O
        % Create objects for reading a video from a file, drawing the tracked
        % objects in each frame, and playing the video.
        
        
        % Create a video file reader.
        obj.reader = ipcam(camList(1,:),'admin','admin');
        %obj.reader = ipcam(camList(labindex,:),'admin','admin');
        %obj.reader = vision.VideoFileReader('atrium.avi');
                
        % Create two video players, one to display the video,
        % and one to display the foreground mask.
        obj.videoPlayer = vision.VideoPlayer('Position', [20, 400, 700, 400]);
        obj.maskPlayer = vision.VideoPlayer('Position', [740, 400, 700, 400]);
        
        % Create system objects for foreground detection and blob analysis
        
        % The foreground detector is used to segment moving objects from
        % the background. It outputs a binary mask, where the pixel value
        % of 1 corresponds to the foreground and the value of 0 corresponds
        % to the background. 
        
        obj.detector = vision.ForegroundDetector('NumGaussians', 5, ...
            'NumTrainingFrames', 1000, 'MinimumBackgroundRatio', 0.1);
        
        % Connected groups of foreground pixels are likely to correspond to moving
        % objects.  The blob analysis system object is used to find such groups
        % (called 'blobs' or 'connected components'), and compute their
        % characteristics, such as area, centroid, and the bounding box.
        
        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 400);
    end

%% Initialize Tracks
% The |initializeTracks| function creates an array of tracks, where each
% track is a structure representing a moving object in the video. The
% purpose of the structure is to maintain the state of a tracked object.
% The state consists of information used for detection to track assignment,
% track termination, and display. 
%
% The structure contains the following fields:
%
% * |id| :                  the integer ID of the track
% * |bbox| :                the current bounding box of the object; used
%                           for display
% * |kalmanFilter| :        a Kalman filter object used for motion-based
%                           tracking
% * |age| :                 the number of frames since the track was first
%                           detected
% * |totalVisibleCount| :   the total number of frames in which the track
%                           was detected (visible)
% * |consecutiveInvisibleCount| : the number of consecutive frames for 
%                                  which the track was not detected (invisible).
% * |CreationTime| :        When the blob was created
% * |LastUpdatedTime| :     When the blob was last updated
% * Both the times are captured using the clock function
%
% Noisy detections tend to result in short-lived tracks. For this reason,
% the example only displays an object after it was tracked for some number
% of frames. This happens when |totalVisibleCount| exceeds a specified 
% threshold.    
%
% When no detections are associated with a track for several consecutive
% frames, the example assumes that the object has left the field of view 
% and deletes the track. This happens when |consecutiveInvisibleCount|
% exceeds a specified threshold. A track may also get deleted as noise if 
% it was tracked for a short time, and marked invisible for most of the of 
% the frames.        

    function tracks = initializeTracks()
        % create an empty array of tracks
        tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {}, ...
            'CreationTime', {}, ...
            'LastUpdatedTime', {} );
    end

%% Read a Video Frame
% Read the next video frame from the IP cam feed.
    function frame = readFrame()
        frame = snapshot(obj.reader);
    end

%% Detect Objects
% The |detectObjects| function returns the centroids and the bounding boxes
% of the detected objects. It also returns the binary mask, which has the 
% same size as the input frame. Pixels with a value of 1 correspond to the
% foreground, and pixels with a value of 0 correspond to the background.   
%
% The function performs motion segmentation using the foreground detector. 
% It then performs morphological operations on the resulting binary mask to
% remove noisy pixels and to fill the holes in the remaining blobs.  

    function [centroids, bboxes, mask] = detectObjects(frame)
        
        % Detect foreground.
        frame2 = impyramid(frame, 'reduce');
        frame3 = impyramid(frame2, 'reduce');
        frame4 = impyramid(frame3, 'expand');
        frame = impyramid(frame4, 'expand');
        mask = obj.detector.step(frame);
        
        % Apply morphological operations to remove noise and fill in holes.
        %mask = imopen(mask, strel('rectangle', [5,5]));
        
        %mask = imclose(mask, strel('rectangle', [15, 15])); 
        mask=medfilt2(mask);
        mask = imclose(mask, strel('disk', 10,8));
        mask = imfill(mask, 'holes');
        
        % Perform blob analysis to find connected components.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
    end

%% Predict New Locations of Existing Tracks
% Use the Kalman filter to predict the centroid of each track in the
% current frame, and update its bounding box accordingly.

    function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;
            
            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);
            
            % Shift the bounding box so that its center is at 
            % the predicted location.
            predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
    end

%% Assign Detections to Tracks
% Assigning object detections in the current frame to existing tracks is
% done by minimizing cost. The cost is defined as the negative
% log-likelihood of a detection corresponding to a track.  
%
% The algorithm involves two steps: 
%
% Step 1: Compute the cost of assigning every detection to each track using
% the |distance| method of the |vision.KalmanFilter| System object. The 
% cost takes into account the Euclidean distance between the predicted
% centroid of the track and the centroid of the detection. It also includes
% the confidence of the prediction, which is maintained by the Kalman
% filter. The results are stored in an MxN matrix, where M is the number of
% tracks, and N is the number of detections.   
%
% Step 2: Solve the assignment problem represented by the cost matrix using
% the |assignDetectionsToTracks| function. The function takes the cost 
% matrix and the cost of not assigning any detections to a track.  
%
% The value for the cost of not assigning a detection to a track depends on
% the range of values returned by the |distance| method of the 
% |vision.KalmanFilter|. This value must be tuned experimentally. Setting 
% it too low increases the likelihood of creating a new track, and may
% result in track fragmentation. Setting it too high may result in a single 
% track corresponding to a series of separate moving objects.   
%
% The |assignDetectionsToTracks| function uses the Munkres' version of the
% Hungarian algorithm to compute an assignment which minimizes the total
% cost. It returns an M x 2 matrix containing the corresponding indices of
% assigned tracks and detections in its two columns. It also returns the
% indices of tracks and detections that remained unassigned. 

    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        nTracks = length(tracks);
        nDetections = size(centroids, 1);
        
        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end
        
        % Solve the assignment problem.
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
    end

%% Update Assigned Tracks
% The |updateAssignedTracks| function updates each assigned track with the
% corresponding detection. It calls the |correct| method of
% |vision.KalmanFilter| to correct the location estimate. Next, it stores
% the new bounding box, and increases the age of the track and the total
% visible count by 1. Finally, the function sets the invisible count to 0. 

    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);
            
            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;
            
            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
            
            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
    end

%% Update Unassigned Tracks
% Mark each unassigned track as invisible, and increase its age by 1.

    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
    end

%% Delete Lost Tracks
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. It also deletes recently created tracks
% that have been invisible for too many frames overall. 

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end
        
        invisibleForTooLong = 20;
        ageThreshold = 8;
        
        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;
        
        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
        
        % Delete lost tracks.
        % @here
        lostTracks = tracks(lostInds);
        if (size(lostTracks) >= 1)
            for i = 1:size(lostTracks)
                x = (lostTracks(i).bbox(1) + lostTracks(i).bbox(3))/2;
                y = (lostTracks(i).bbox(2) + lostTracks(i).bbox(4))/2;
                removeString = strcat('{"age":"LOST", "connectionType": "DATASOURCE",', '"id":', num2str(lostTracks(i).id), ',"origin": {"x":', num2str(x), ',"y":', num2str(y), ...
                    ',"z": 0}, "orientation": {"x":0, "y":0, "z":0, "theta":0}, "source":"MATLAB", "updateTime":0, "creationTime":0,', ...
                    '"boundingBox":{"x":', num2str(lostTracks(i).bbox(1)), ', "y":', num2str(lostTracks(i).bbox(2)), ...
                    ',"width":', num2str(lostTracks(i).bbox(3)), ',"height":', num2str(lostTracks(i).bbox(4)), '}}&');
                fwrite(serverObj, removeString);
            end
        end
        tracks = tracks(~lostInds);
    end

%% Create New Tracks
% Create new tracks from unassigned detections. Assume that any unassigned
% detection is a start of a new track. In practice, you can use other cues
% to eliminate noisy detections, such as size, location, or appearance.

    function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);
        
        for i = 1:size(centroids, 1)
            
            centroid = centroids(i,:);
            bbox = bboxes(i, :);
            
            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 100);
            
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0, ...
                'CreationTime', clock, ...
                'LastUpdatedTime', clock);
            
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
            
            % Increment the next id.
            nextId = nextId + 1;
        end
    end

%% Display Tracking Results
% The |displayTrackingResults| function draws a bounding box and label ID 
% for each track on the video frame and the foreground mask. It then 
% displays the frame and the mask in their respective video players. 

    function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask = uint8(repmat(mask, [1, 1, 3])) .* 255;
        
        minVisibleCount = 8;
        if ~isempty(tracks)
              
            % Noisy detections tend to result in short-lived tracks.
            % Only display tracks that have been visible for more than 
            % a minimum number of frames.
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);
            
            % Sends blob data to server
            sendDataToServer();
            
            % Display the objects. If an object has not been detected
            % in this frame, display its predicted bounding box.
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox);
                
                % Get ids.
                ids = int32([reliableTracks(:).id]);
                
                % Create labels for objects indicating the ones for 
                % which we display the predicted rather than the actual 
                % location.
                labels = cellstr(int2str(ids'));
                predictedTrackInds = ...
                    [reliableTracks(:).consecutiveInvisibleCount] > 0;
                isPredicted = cell(size(labels));
                isPredicted(predictedTrackInds) = {' predicted'};
                labels = strcat(labels, isPredicted);
                
                % Draw the objects on the frame.
                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);
                
                % Draw the objects on the mask.
                mask = insertObjectAnnotation(mask, 'rectangle', ...
                    bboxes, labels);
            end
        end
        
        % Display the mask and the frame.
        obj.maskPlayer.step(mask);        
        obj.videoPlayer.step(frame);
    end

%% Send Blob Data To Server
% The |sendDataToServer| function sends new and old blob information 
% to the server. Each |tracks| carries one blob data. 
% Blob data is sent as two JSON arrays. One for old blob data and one for
% newly created blob data. Each frame blob information is terminated with 
% an ampersand.

    function sendDataToServer()
        
            minVisibleCount = 8;
        
            % Get the indices of new tracks.
            % New tracks are on which have been visible for exactly
            % minVisibleCount + 1 frames
            NewTrackInds = ...
                [tracks(:).totalVisibleCount] == minVisibleCount + 1;
            NewTracks = tracks(NewTrackInds);
            
            for k = 1:length(NewTracks)
                NewTracks(k).LastUpdatedTime = clock;
                NewTracks(k).id;
                x = (NewTracks(k).bbox(1) + NewTracks(k).bbox(3))/2;
                y = (NewTracks(k).bbox(2) + NewTracks(k).bbox(4))/2;
                NewTracks(k).CreationTime;
                NewTracks(k).LastUpdatedTime;
                NewTracks(k).bbox(1);
                NewTracks(k).bbox(2);
                NewTracks(k).bbox(3);
                NewTracks(k).bbox(4);                
                
                newString = strcat('{"age":"NEW", "connectionType": "DATASOURCE",', '"id":', num2str(NewTracks(k).id), ',"origin": {"x":', num2str(x), ',"y":', num2str(y), ...
                ',"z": 0}, "orientation": {"x":0, "y":0, "z":0, "theta":0}, "source":"MATLAB", "updateTime":0, "creationTime":0,', ...
                '"boundingBox":{"x":', num2str(NewTracks(k).bbox(1)), ', "y":', num2str(NewTracks(k).bbox(2)), ...
                ',"width":', num2str(NewTracks(k).bbox(3)), ',"height":', num2str(NewTracks(k).bbox(4)), '}}&');
                fwrite(serverObj, newString);
                %if(k ~= length(NewTracks))
                %    newString = strcat(newString, ',');
                %end                
            end
            
            
            % Get the indices of old tracks.
            % Old tracks are on which have been visible for 
            % grater than minVisibleCount + 1 frames
            OldTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount + 1;
            OldTracks = tracks(OldTrackInds);
            
            for k = 1:length(OldTracks)
                OldTracks(k).LastUpdatedTime = clock;
                OldTracks(k).id;
                x = (OldTracks(k).bbox(1) + OldTracks(k).bbox(3))/2;
                y = (OldTracks(k).bbox(2) + OldTracks(k).bbox(4))/2;
                OldTracks(k).CreationTime;
                OldTracks(k).LastUpdatedTime;
                OldTracks(k).bbox(1);
                OldTracks(k).bbox(2);
                OldTracks(k).bbox(3);
                OldTracks(k).bbox(4);
                
                oldString = strcat('{"age":"OLD", "connectionType": "DATASOURCE",', '"id":', num2str(OldTracks(k).id), ',"origin": {"x":', num2str(x), ',"y":', num2str(y), ...
                ',"z": 0}, "orientation": {"x":0, "y":0, "z":0, "theta":0}, "source":"MATLAB", "updateTime":0, "creationTime":0,', ...
                '"boundingBox":{"x":', num2str(OldTracks(k).bbox(1)), ', "y":', num2str(OldTracks(k).bbox(2)), ...
                ',"width":', num2str(OldTracks(k).bbox(3)), ',"height":', num2str(OldTracks(k).bbox(4)), '}}&');
               
                fwrite(serverObj, oldString);
                %if(k ~= length(OldTracks))
                %    oldString = strcat(oldString, ',');
                %end
            end
            
            %fwrite(serverObj, output);
            %fwrite(serverObj,'&');
    end

%% Summary
% This is a motion-based system for detecting and 
% tracking multiple moving objects. 

% The tracking is solely based on motion with the assumption that all 
% objects move in a straight line with constant speed.
% When the motion of an object significantly deviates from this model, the
% example may produce tracking errors.  
%
% The likelihood of tracking errors can be reduced by using a more complex
% motion model, such as constant acceleration, or by using multiple Kalman
% filters for every object. Also, one can incorporate other cues for
% associating detections over time, such as size, shape, and color. 

end



