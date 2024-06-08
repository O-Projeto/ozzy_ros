from elevenlabs import play,save
from elevenlabs.client import ElevenLabs

client = ElevenLabs(
  api_key="sk_2db572db3a5c99e4924778b15548a1263a57255a331af6dc", # Defaults to ELEVEN_API_KEY
)

audio = client.generate(
  text="Olá queridos e queridas do Brasil, vocês estão prontos para encher meu saco na RCX ? Venha conversar comigo nesse evento único cheio de robôs, explosões e caveiras muito loucas",
  voice="Vpv1YgvVd6CHIzOTiTt8",
  model="eleven_multilingual_v2"
)
save(audio,'rcx.wav')
play(audio)