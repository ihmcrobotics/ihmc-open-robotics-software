function deletewithsemaphores(fileList)
%DELETEWITHSEMAPHORES  Delete files using semaphors.
%		DELETEWITHSEMAPHORES(FILELIST) deletes the files in cell array FILELIST
%		using semaphores.
%
%		Example:
%		fileList = {'test1.txt', 'text2.txt'};
%		deletewithsemaphores(fileList);
%
%		Markus Buehren
%		Last modified 21.12.2008
%
%		See also SETFILESEMAPHORE, REMOVEFILESEMAPHORE.

checkWaitTime = 0.1;

if ischar(fileList)
	fileList = {fileList};
end

for fileNr = 1:length(fileList)
	sem = setfilesemaphore(fileList{fileNr});
	for attemptNr = 1:10
		try
			if existfile(fileList{fileNr})
				delete(fileList{fileNr}); %% file access %%
			end
			break
		catch
			pause(checkWaitTime);
		end
	end
	removefilesemaphore(sem);
end
