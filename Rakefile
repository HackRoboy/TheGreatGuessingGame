require 'rake/testtask'

task default: :build

desc 'Builds the App.'
task :build do
  sh 'java'
end

task :format do
  options = []
  options.push '--replace' if ENV['repair']
  sh "gherkin_format #{options.join ' '} features/*.feature"
end
