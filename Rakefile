require 'rake/testtask'

task default: :build

desc 'Builds the App.'
task gen: 'DialogSystem/DialogSystem.jar'
task gen: 'build/extraction.weights'

task :format do
  options = []
  options.push '--replace' if ENV['repair']
  sh "gherkin_format #{options.join ' '} features/*.feature"
end

task run: 'DialogSystem/DialogSystem.jar' do
  sh run_command()
end

def run_command()
  "#{ENV['JAVA_HOME']}/bin/java -cp DialogSystem:DialogSystem/resources:DialogSystem/DialogSystem.jar:#{FileList['DialogSystem/lib/*.jar'].join ':'} de.roboy.dialog.DialogSystem"
end

directory 'build'

file 'build/extraction.weights'
file 'build/extraction.weights' do
  mkdir_p 'build'
  sh "curl -O http://pjreddie.com/media/files/extraction.weights --output extraction.weights"  
  sh 'mv extraction.weights build'
end

file 'DialogSystem/DialogSystem.jar' => FileList['DialogSystem/**/*.java'] do
  cd('DialogSystem') do 
    mkdir_p 'package'
    sh 'cp -r resources ..'
    sh "#{ENV['JAVA_HOME']}/bin/javac #{FileList['src/**/*.java']} -cp #{FileList['lib/*.jar'].join ':'} -d package"
    sh "#{ENV['JAVA_HOME']}/bin/jar cfm DialogSystem.jar Manifest.txt -C package/ ."
  end
end

desc 'Tests the Application'
task :test => :gen do
  ENV['sut'] = run_command
 
  options = []
  options << '--stop' if ENV['stop']
  options << '--tags ~skip'
  sh "PYTHONPATH=src behave #{options * ' '} test"
end

task :game => :gen do
  ENV['command'] = run_command
  ENV['PYTHONPATH'] = 'src'
  
  #sh 'PYTHONPATH=src python3 src/roboy/guessing_game.py'
  sh 'python3 src/roboy/guessing_game.py'
end
