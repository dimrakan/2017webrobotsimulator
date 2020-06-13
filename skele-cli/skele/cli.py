"""
skele

Usage:
  skele tf --ip=<ip> --secret=<secret>
  skele tf --ip=<ip> --secret_fpath=<secet_fpath>
  skele sub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret> 
  skele sub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret_fpath> 
  skele sub --remote=<remote>  --ip=<ip> --secret=<secret>
  skele sub --remote=<remote>  --ip=<ip> --secret_fpath=<secret_fpath>
  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret>
  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret_fpath=<secret_fpath>
  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret> --type=<type>
  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret_fpath=<secret_fpath> --type=<type>
  skele pub --remote=<remote>  --ip=<ip> --secret=<secret>
  skele pub --remote=<remote>  --ip=<ip> --secret_fpath=<secret_fpath>
  skele srv --ip=<ip> --secret=<secret> --remote=<remote> 
  skele srv --ip=<ip> --secret_fpath=<secret_fpath> --remote=<remote> 
  skele action --ip=<ip> --secret=<secret> --remote=<remote> 
  skele action --ip=<ip> --secret_fpath=<secret_fpath> --remote=<remote> 

 
Examples:
  skele tf --ip=83.21.96.15:8115 --secret=Randka3nsak3nda
  skele pub --ip=83.212.96.15:8119 --sercet_fpath=/home/test/secret.txt --remote=/robot/scan --local=/scan 

"""


from inspect import getmembers, isclass

from docopt import docopt
import time
from . import __version__ as VERSION


def main():
    """Main CLI entrypoint."""
    import skele.commands
    options = docopt(__doc__, version=VERSION)
    for (k, v) in options.items(): 
        if hasattr(skele.commands, k) and v:
            module = getattr(skele.commands, k)
            skele.commands = getmembers(module, isclass)
            print(skele.commands) 
            command = [command[1] for command in skele.commands if command[0] != 'Base'][6]
            print(command)
            command = command(ip=options['--ip'].split(':')[0],port=options['--ip'].split(':')[1],remote_topic=options['--remote'],local_topic=options['--local'],secret=options['--secret'],topic_type=options['--type'],secret_fpath='/home/test/aaa.txt') #,req=options['--request'])
            command.run()
#ip=options['--ip'].split(':')[0],port=options['--ip'].split(':')[1],remote_topic=options['--remote'],local_topic=options['--local'],secret=options['--secret'],req=options['--request'])
            while True:
              if command.terminated():
                break
              time.sleep(3) 
