%% Parse an MPS/QPS file
% see http://infohost.nmt.edu/~borchers/readmps.html
function problem = readmps(filename)
%
% Initiailize all of the output record fields
%
problem.name='';
problem.objsense='MINIMIZE';         % Default is to minimize.
problem.objname='';
problem.refrow='';
problem.rownames=cell(0);
problem.rowtypes=cell(0);
problem.columnnames=cell(0);
problem.boundnames=cell(0);
problem.rhsnames=cell(0);
problem.rangenames=cell(0);
problem.lbnds=sparse([]);
problem.ubnds=sparse([]);
problem.rhs=sparse([]);
problem.ranges=sparse([]);
problem.bintflags=sparse([]);    % For integer variables in bounds
problem.intflags=sparse([]);     % For integer variables in INTORG
problem.sos1flags=sparse([]);    % For SOS1 variables by SOSORG/SOSEND
problem.sos2flags=sparse([]);    % For SOS2 variables by SOSORG/SOSEND
problem.sos3flags=sparse([]);    % For SOS3 variables by SOSORG/SOSEND
problem.Q=sparse([]);
%
% Initialize variables used inside this routine.
%
rowcount=0;
colcount=0;
boundcount=0;
rhscount=0;
rangecount=0;
intflag=0;
sos1flag=0;
sos2flag=0;
sos3flag=0;
%
% Some size limit parameters.
%
maxm=100000;
maxn=5000000;
maxrhs=100;
maxbounds=100;
maxranges=100;
%
% Some tables.
%
rowtable=newtable(maxm);
coltable=newtable(maxn);
boundtable=newtable(maxbounds);
rhstable=newtable(maxrhs);
rangetable=newtable(maxranges);
%
% Open the file 
%
fid=fopen(filename,'r');
if (fid==-1)
  error('Could not open input file');
end
%
% Get the first line.
%
[line,f]=getfields(fid);
%
% The main loop goes through the sections of the file.  
%
while (1==1)
%
% Check for missing section card.
%
  if (line(1)==' ')
%
% Missing a section card.
%
    warning('Missing Section Card');
    [line,f]=getfields(fid);
  else
%
% We have a section card.  Process that section.
%
%
% Handle ENDATA
%
    if (strcmp(f{1},'ENDATA')==1)
      problem.rowtable=rowtable;
      problem.coltable=coltable;
      problem.boundtable=boundtable;
      problem.rhstable=rhstable;
      problem.rangetable=rangetable;
      return;
    end;
%
% Handle the NAME section.
%
    if (strcmp(f{1},'NAME')==1)
%
% NAME might be empty!
%
      if (length(f) > 1)
        problem.name=f{2};
      end
      [line,f]=getfields(fid);
      continue;
    end

%
% Handle the OBJSENSE section.
%
    if (strcmp(f{1},'OBJSENSE')==1)
      [line,f]=getfields(fid);
      problem.objsense=f{1};
      [line,f]=getfields(fid);
      continue;
    end

%
% Handle the REFROW section.
%
    if (strcmp(f{1},'REFROW')==1)
      [line,f]=getfields(fid);
      problem.refrow=f{1};
      [line,f]=getfields(fid);
      continue;
    end

%
% Handle the OBJNAME section.
%
    if (strcmp(f{1},'OBJNAME')==1)
      [line,f]=getfields(fid);
      problem.objname=f{1};
      [line,f]=getfields(fid);
      continue;
    end

%
% Handle the ROWS Section.
%
    if (strcmp(f{1},'ROWS')==1)
      [line,f]=getfields(fid);
      while ((length(line)==0) |  (line(1)==' '))
        rowcount=rowcount+1;
        problem.rownames{rowcount}=f{2};
        problem.rowtypes{rowcount}=upper(f{1});
	rowtable=addtotable(rowtable,f{2},rowcount);
	[line,f]=getfields(fid);
      end
%
% While we're here, initialize problem.A as a sparse array with
% the correct number of rows.  We don't yet know the number of
% columns.
%
      problem.A=sparse(rowcount,rowcount*10);
%
% Move on to the next section.  
%
      continue;
    end


%
% Handle the COLUMNS section.
%
    if (strcmp(f{1},'COLUMNS')==1)
      [line,f]=getfields(fid);
      while ((length(line)==0) |  (line(1)==' '))

%
% First, check for special INTORG/INTEND 'MARKER' records.
%
        if ((strcmp(f{2},'''MARKER''')==1) & (strcmp(f{3},'''INTORG''')==1))
	  intflag=1;
	  [line,f]=getfields(fid);
	  continue
	end
        if ((strcmp(f{2},'''MARKER''')==1) & (strcmp(f{3},'''INTEND''')==1))
	  intflag=0;
	  [line,f]=getfields(fid);
	  continue
	end

%
% Next, check for special SOSORG/SOSEND 'MARKER' records.
%
        if (length(f) >= 4)
        if ((strcmp(f{3},'''MARKER''')==1) & (strcmp(f{4},'''SOSORG''')==1))
	  if ((strcmp(f{1},'S1')==1))
	    sos1flag=1;
          end
	  if ((strcmp(f{1},'S2')==1))
	    sos2flag=1;
          end
	  if ((strcmp(f{1},'S3')==1))
	    sos3flag=1;
          end

	  [line,f]=getfields(fid);
	  continue
	end
        end
        if (length(f)>=3)
          if ((strcmp(f{2},'''MARKER''')==1) & (strcmp(f{3},'''SOSEND''')==1))
	    sos1flag=0;
            sos2flag=0;
	    sos3flag=0;    
	    [line,f]=getfields(fid);
	    continue
	  end
        end
%
% Handle regular COLUMNS records.
%

	if (length(f)==3)
          colname=f{1};
          lastcolname=f{1};
          row1=f{2};
          entry1=str2num(f{3});
	  row2='';
	  entry2=0;
	end
	if (length(f)==5)
          colname=f{1};
          lastcolname=f{1};
          row1=f{2};
          entry1=str2num(f{3});
          row2=f{4};
          entry2=str2num(f{5});
	end
	if (length(f)==2)
	  colname=lastcolname;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2='';
	  entry2=0;
	end
	if (length(f)==4)
	  colname=lastcolname;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2=f{3};
	  entry2=str2num(f{4});
	end

	if (length(f)>5)
	  error('Too many fields in record');
	end

        if (length(f) > 3)
%
% Two entries in this record.
%
%	  [colname ' ' row1 ' ' num2str(entry1) ' ' row2 ' ' num2str(entry2)]
%    fflush(1);
          newcolno=tablelookup(coltable,colname);
          if (isnan(newcolno))
	    colcount=colcount+1;
	    coltable=addtotable(coltable,colname,colcount);
	    colno=colcount;
            problem.columnnames{colcount}=colname;
	    problem.intflags(colcount)=intflag;
            problem.sos1flags(colcount)=sos1flag;
            problem.sos2flags(colcount)=sos2flag;
            problem.sos3flags(colcount)=sos3flag;
	  else
	    if (newcolno ~= colno)
	      warning(['Columns records out of order!']);
	    end
	    colno=newcolno;
	  end

	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    error(['COLUMNS Entry specifies row that does not' ...
		   ' exist']);
	  end

	  problem.A(rowno,colno)=entry1;

	  rowno=tablelookup(rowtable,row2);
	  if (isnan(rowno))
	    error(['COLUMNS Entry specifies row that does not' ...
		   ' exist']);
	  end

	  problem.A(rowno,colno)=entry2;

	else
%
% One entry in this record.
%
%	  [colname ' ' row1 ' ' num2str(entry1)]
%    fflush(1);

          newcolno=tablelookup(coltable,colname);
          if (isnan(newcolno))
	    colcount=colcount+1;
	    coltable=addtotable(coltable,colname,colcount);
	    colno=colcount;
            problem.columnnames{colcount}=colname;
	    problem.intflags(colcount)=intflag;
	  else
	    if (newcolno ~= colno)
	      warning(['Columns records out of order!']);
	    end
	    colno=newcolno;
	  end
	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    line
	    error(['COLUMNS Entry specifies row that does not' ...
		   ' exist']);
	  end
	  
	  problem.A(rowno,colno)=entry1;
	end
%
% Move on to the next record.
%
        [line,f]=getfields(fid);
      end

%
% Move on to the next section.
%
%
% Now that we know the number of rows and columns, we know the
% correct dimensions for A.  Make sure that it has this size.
%

      [m,n]=size(problem.A);
      problem.A=problem.A(1:rowcount,1:colcount);
      if (m<rowcount)
	problem.A(rowcount,1)=0;
      end
      if (n<colcount)
	problem.A(1,colcount)=0;
      end
%
% Continue on to the next section.  
%
      continue;
    end



%
% Handle the QSECTION section.  This is called different things by 
% different people, e.g. QMATRIX or QUADOBJ.
%
    if ((strcmp(f{1},'QSECTION')==1) | (strcmp(f{1},'QUADOBJ')==1) ...
	| (strcmp(f{1},'QMATRIX')))
      problem.Q=sparse(colcount,colcount);
      [line,f]=getfields(fid);
      while ((length(line)==0) |  (line(1)==' '))

%
% Handle an entry. 
%
	if (length(f)==3)
          colname1=f{1};
          colname2=f{2};
          entry=str2num(f{3});
	  col1=tablelookup(coltable,colname1);
	  col2=tablelookup(coltable,colname2);
	  if (isnan(col1))
	    error('Invalid column name in QSECTION.');
	  end
	  if (isnan(col2))
	    error('Invalid column name in QSECTION.');
	  end
	  problem.Q(col1,col2)=entry;
        else
	  error('Wrong number of fields in QSECTION record');
	end
%
% Move on to the next record.
%
        [line,f]=getfields(fid);
      end
%
% Continue on to the next section.  
%
      continue;
    end


%
% Handle the RHS section.
%
    if (strcmp(f{1},'RHS')==1)
      [line,f]=getfields(fid);
%
% A default RHS name is used here in case the authors of the
% problem have not provided any RHS name.  
%
      lastrhsname='DEFAULTRHS';
      while ((length(line)==0) |  (line(1)==' '))
	if (length(f)==3)
          rhsname=f{1};
          lastrhsname=f{1};
          row1=f{2};
          entry1=str2num(f{3});
	  row2='';
	  entry2=0;
	end
	if (length(f)==5)
          rhsname=f{1};
          lastrhsname=f{1};
          row1=f{2};
          entry1=str2num(f{3});
          row2=f{4};
          entry2=str2num(f{5});
	end
	if (length(f)==2)
	  rhsname=lastrhsname;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2='';
	  entry2=0;
	end
	if (length(f)==4)
	  rhsname=lastrhsname;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2=f{3};
	  entry2=str2num(f{4});
	end

%
% Add the RHS information to the tables.
%
        if (length(f) > 3)
%
% We have two entries in this record
%
%	  [rhsname ' ' row1 ' ' num2str(entry1) ' ' row2 ' ' num2str(entry2)]

          rhsno=tablelookup(rhstable,rhsname);
	  if (isnan(rhsno))
	    rhscount=rhscount+1;
	    problem.rhsnames{rhscount}=rhsname;
	    rhstable=addtotable(rhstable,rhsname,rhscount);
	    rhsno=rhscount;
	    problem.rhs(rowcount,rhscount)=0;
	  end

	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    error(['RHS Entry specifies row that does not' ...
		   ' exist']);
	  end

	  problem.rhs(rowno,rhsno)=entry1;

	  rowno=tablelookup(rowtable,row2);
	  if (isnan(rowno))
	    error(['RHS Entry specifies row that does not' ...
		   ' exist']);
	  end
	  problem.rhs(rowno,rhsno)=entry2;

	else
%
% We have one entry in this record
%
%	  [rhsname ' ' row1 ' ' num2str(entry1)]

          rhsno=tablelookup(rhstable,rhsname);
	  if (isnan(rhsno))
	    rhscount=rhscount+1;
	    problem.rhsnames{rhscount}=rhsname;
	    rhstable=addtotable(rhstable,rhsname,rhscount);
	    rhsno=rhscount;
	    problem.rhs(rowcount,rhscount)=0;
	  end

	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    error(['RHS Entry specifies row that does not' ...
		   ' exist']);
	  end
	  problem.rhs(rowno,rhsno)=entry1;

	end
%
% Move on to the next record.
%
        [line,f]=getfields(fid);
      end
%
% Move on to the next section.
%
      continue;
    end





%
% Handle the RANGES section.
%
    if (strcmp(f{1},'RANGES')==1)
      [line,f]=getfields(fid);
      lastrangename='DEFAULTRNG';
      while ((length(line)==0) |  (line(1)==' '))
	if (length(f)==3)
          rangename=f{1};
          lastrangename=f{1};
          row1=f{2};
          entry1=str2num(f{3});
	  row2='';
	  entry2=0;
	end
	if (length(f)==5)
          rangename=f{1};
          lastrangename=f{1};
          row1=f{2};
          entry1=str2num(f{3});
          row2=f{4};
          entry2=str2num(f{5});
	end
	if (length(f)==2)
	  rangename=lastrangename;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2='';
	  entry2=0;
	end
	if (length(f)==4)
	  rangename=lastrangename;
	  row1=f{1};
	  entry1=str2num(f{2});
	  row2=f{3};
	  entry2=str2num(f{4});
	end

%
% Add the RANGE information to the tables.
%
        if (length(f) > 3)
%
% We have two entries in this record
%

          rangeno=tablelookup(rangetable,rangename);
	  if (isnan(rangeno))
	    rangecount=rangecount+1;
	    problem.rangenames{rangecount}=rangename;
	    rangetable=addtotable(rangetable,rangename,rangecount);
	    rangeno=rangecount;
	    %
	    % Initialize default range values for N, E, L, G constraints
	    %
	    for i=1:rowcount
	      if ((problem.rowtypes{i}=='L') | ...
		  (problem.rowtypes{i}=='G'))
		problem.ranges(i,rangecount)=+Inf;
	      else
		problem.ranges(i,rangecount)=0;
	      end
	    end
	  end

	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    error(['RANGES Entry specifies row that does not' ...
		   ' exist']);
	  end

	  problem.ranges(rowno,rangeno)=entry1;

	  rowno=tablelookup(rowtable,row2);
	  if (isnan(rowno))
	    error(['RANGES Entry specifies row that does not' ...
		   ' exist']);
	  end
	  problem.ranges(rowno,rangeno)=entry2;

	else
%
% We have one entry in this record
%

          rangeno=tablelookup(rangetable,rangename);
	  if (isnan(rangeno))
	    rangecount=rangecount+1;
	    problem.rangenames{rangecount}=rangename;
	    rangetable=addtotable(rangetable,rangename,rangecount);
	    rangeno=rangecount;
	  end

	  rowno=tablelookup(rowtable,row1);
	  if (isnan(rowno))
	    error(['RANGES entry specifies row that does not' ...
		   ' exist']);
	  end
	  problem.ranges(rowno,rangeno)=entry1;

	end
%
% Move on to the next record.
%
        [line,f]=getfields(fid);
      end
%
% Move on to the next section.
%
      continue;
    end

%
% Handle the BOUNDS section. 
%
    if (strcmp(f{1},'BOUNDS')==1)
      [line,f]=getfields(fid);
      boundname='';
      while ((length(line)==0) |  (line(1)==' '))

	if (length(f) < 2)
	  line
	  error('Too few fields in BOUNDS record');
	end
	if (length(f) > 4)
	  line
	  error('Too many fields in BOUNDS record');
	end

	boundtype=f{1};

	if (length(f)==4)
	  boundname=f{2};
          colname=f{3};
          entry=str2num(f{4});
	end

%
% Two fields is a special case.
%
	if (length(f)==2)
%
% This is only legal if no boundname is given and the bound type is
% FR or BV.
%
          if ((strcmp(f{1},'BV')==1) | (strcmp(f{1},'FR')))
	    colname=f{2};
	    entry=0;
	  else
	    line
	    error('Invalid entry in BOUNDS section.');
	  end
	end

%
% 3 fields is another special case.
%
	if (length(f)==3)
%
% This could either be a BV or FR bound with no entry, or it could
% be a conventional bound, with no bound name given.
%
          if ((strcmp(f{1},'BV')==1) | (strcmp(f{1},'FR')))
	    boundname=f{2};
	    colname=f{3};
	    entry=0;
	  else
	    colname=f{2};
	    entry=str2num(f{3});
	  end
	end
	
	boundno=tablelookup(boundtable,boundname);
	if (isnan(boundno))
	  boundcount=boundcount+1;
	  boundtable=addtotable(boundtable,boundname,boundcount);
	  problem.lbnds(boundcount,1:colcount)=0;
	  problem.ubnds(boundcount,1:colcount)=+Inf;
%
% Default upper bound for integer variables is 1.
%
          for i=1:colcount
            if (problem.intflags(i)==1)
	      problem.ubnds(boundcount,i)=1;
	    end
          end
	  problem.bintflags(boundcount,1:colcount)=0;
          boundno=boundcount;
	  problem.boundnames{boundno}=boundname;
	end
	colno=tablelookup(coltable,colname);
	if (isnan(colno))
	  line
	  error('Invalid column in BOUNDS section');
	end

%
% Handle the different cases.
%
	if (upper(boundtype)=='LO')
	  problem.lbnds(boundno,colno)=entry;
	end
	if (upper(boundtype)=='UP')
	  problem.ubnds(boundno,colno)=entry;
	end
	if (upper(boundtype)=='FX')
	  problem.lbnds(boundno,colno)=entry;
	  problem.ubnds(boundno,colno)=entry;
	end
	if (upper(boundtype)=='FR')
	  problem.lbnds(boundno,colno)=-Inf;
	  problem.ubnds(boundno,colno)=+Inf;
	end
	if (upper(boundtype)=='MI')
	  problem.lbnds(boundno,colno)=-Inf;
	end
	if (upper(boundtype)=='PL')
	  problem.ubnds(boundno,colno)=+Inf;
	end

	if (upper(boundtype)=='LI')
	  problem.lbnds(boundno,colno)=entry;
	  problem.bintflags(boundno,colno)=1;
	end

	if (upper(boundtype)=='UI')
	  problem.ubnds(boundno,colno)=entry;
	  problem.bintflags(boundno,colno)=1;
	end

	if (upper(boundtype)=='BV')
	  problem.lbnds(boundno,colno)=0;
	  problem.ubnds(boundno,colno)=1;
	  problem.bintflags(boundno,colno)=1;
	end

%
% Get the next record.
%
        [line,f]=getfields(fid);
      end
      
%
% Move on to the next section.
%
      continue
    end

%
% It's some other type of section.
%
    if (1==1)
      disp('Unhandled section');
      line
      [line,f]=getfields(fid);

      while ((length(line)==0) |  (line(1)==' '))
        [line,f]=getfields(fid);
      end
    end
  end
end

end
