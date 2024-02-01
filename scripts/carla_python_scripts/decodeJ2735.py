#SAE J275 DECODER
import J2735_201603_combined_voices_mr_fix as J2735
import socket
import json
import csv
import binascii as ba
from time import sleep
import readline

#defines a function to generate a user input with information prefilled
def input_with_prefill(prompt, text):
    def hook():
        readline.insert_text(text)
        readline.redisplay()
    readline.set_pre_input_hook(hook)
    result = input(prompt)
    readline.set_pre_input_hook()
    return result


input_filename = input_with_prefill("Captured packet hex: ", "")
output_filename = input_with_prefill("Output json filename: ", "decoded_packet.json")

# f = open(input_filename, 'r')

#remove newlines and print raw hex
data = input_filename
print('hex data = ', data)

try:
  #specify message type inside J2735.py
  decoded_msg = J2735.DSRC.MessageFrame
  #convert from hex using unhexlify then from uper using library
  decoded_msg.from_uper(ba.unhexlify(data))
  #format data into json
  decoded_msg_json = decoded_msg.to_json()

  print('')
  print(decoded_msg_json)
except Exception as err:
  print(f"Unexpected {err=}, {type(err)=}")
  raise



try:
  with open(output_filename, 'w', encoding='utf-8') as f:
    f.write(decoded_msg_json)
    f.close()
    print('\nFormatted json saved to ' + str(output_filename))
except:
  print('\nFailed to format and save json file')

