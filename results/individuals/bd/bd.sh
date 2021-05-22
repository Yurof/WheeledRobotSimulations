yourfilenames=`ls *.log`
for eachfile in $yourfilenames
do
   python ~/Documents/GitHub/Prandroide/Simulationfastsim/controllers/novelty/maze_plot.py $eachfile
   echo $eachfile
done

