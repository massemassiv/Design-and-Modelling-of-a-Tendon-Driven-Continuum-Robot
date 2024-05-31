## KEX resultat

rm(list=ls())

### for loop för att läsa in dat ### 

file_names = Sys.glob("proccesed_data_const_theta50*.csv")
file_names

# konstanter
k = 50 # antalet mätningar i varje punkt
m = 20 # antalet punkter på cirkeln

n = 50*20 # antalet mätningar totalt
#### 

medelFel_lista = list()
i = 1

for(file in file_names){
  
  data = read.csv(file, header=TRUE)
  
  data = data[,-1] # tar bort index kolumnen
  
  vinkel_estimat = data[,c(1,2,3)] # uppmätta vinklar
  vinkel_teori = data[, c(4,5,6)] # teoretiska vinklar
  
  vinkel_diff = vinkel_estimat - vinkel_teori
  
  medelFel_per_punkt = matrix(NA, nrow=m, ncol=3)
  reshape_mat = as.matrix(vinkel_diff) # gör om till matris 
  vinkel_diff_per_punkt = matrix(reshape_mat, nrow=k, ncol=3*m) # om shape
  
  medelFel_per_punkt = colMeans(vinkel_diff_per_punkt) # medel fel i varje punkt
  # första 20 för pitch, andra 20 för yaw, sista 20 för roll
  
  medelFel_matris = matrix(medelFel_per_punkt, nrow=m, ncol=3)
  
  medelFel_lista[[i]] = medelFel_matris
  i = i + 1 
  
}
#medelFel_lista 

## Beräkna medel fel och gör plottar ## 


medelFel_punktvis = matrix(NA, nrow=m, ncol=6)
medelFel_pitch = matrix(NA, nrow=m, ncol=length(file_names))
medelFel_roll = matrix(NA, nrow=m, ncol=length(file_names))
medelFel_yaw = matrix(NA, nrow=m, ncol=length(file_names))


for(i in 1:length(file_names)){
  
  omgang_i = medelFel_lista[[i]]
  
  medelFel_pitch[, i] = omgang_i[,1]
  medelFel_roll[, i] = omgang_i[,3]
  medelFel_yaw[,i] = omgang_i[,2]
  
  
}
medelFel_pitch # bara för pitch

radFelPitch = rowMeans(medelFel_pitch)
radFelRoll = rowMeans(medelFel_roll)
radFelYaw = rowMeans(medelFel_yaw)

library(ggplot2)


## En cirkel
r = 5
theta = seq(0, 2*pi, length=m)
x = r*cos(theta)
y = r*sin(theta)

# Ta abs och minus för felet
radFelPitchAbs = abs(radFelPitch)
radFelRollAbs  = abs(radFelRoll)
radFelYawAbs   = abs(radFelYaw)


df = data.frame(
  x = x, 
  y = y,
  Error = radFelPitchAbs
  
)


cols = c("green", "yellow", "red")


ggplot(df, aes(x,y)) + 
  geom_path(aes(colour = Error), size=1.5) +
  scale_colour_gradientn(colours =cols) + 
  labs(x ="")+
  labs(y="")+
  labs(title = "Average pitch error on 10 runs for different rotational angles")+
  theme(plot.title = element_text(hjust = 0.5))

df2  = data.frame(
  x = x, 
  y = y,
  Error = radFelRollAbs
  
)

ggplot(df2, aes(x,y)) + 
  geom_path(aes(colour = Error), size=1.5) +
  scale_colour_gradientn(colours =cols) + 
  labs(x ="")+
  labs(y="")+
  labs(title = "Average roll error on 10 runs for different rotational angles")+
  theme(plot.title = element_text(hjust = 0.5))

###  Kvantil plot för pitch

kvantiler = matrix(NA, nrow=2, ncol=m)


for(i in 1:m){
  kvantiler[,i] = quantile(medelFel_pitch[i,],prob=c(0.025, 0.975))
  
}

pd = position_dodge(0)

df3 = data.frame(
  gamma=gamma,
  pitch_err = radFelPitch
)

ggplot(df3, aes(gamma,pitch_err)) + 
  geom_errorbar(aes(ymin=kvantiler[1,], ymax=kvantiler[2,]))+
  geom_line(position = pd, colour="Red", size=1.2)+
  geom_point(position = pd, pch=15, size=2.5)+
  labs(x="Rotational angle")+
  labs(y = "Average pitch error") + 
  labs(title = "Average pitch error spread")+
  theme(plot.title = element_text(hjust = 0.5))


## Kvantil För roll

kvantiler = matrix(NA, nrow=2, ncol=m)

for(i in 1:m){
  kvantiler[,i] = quantile(medelFel_roll[i,],prob=c(0.025, 0.975))
  
}

df4 = data.frame(
  gamma=gamma,
  roll_err = radFelRoll
)

ggplot(df4, aes(gamma,roll_err)) + 
  geom_errorbar(aes(ymin=kvantiler[1,], ymax=kvantiler[2,]))+
  geom_line(position = pd, colour="lightblue", size=1.2)+
  geom_point(position = pd, pch=15, size=2.5)+
  labs(x="Rotational angle")+
  labs(y = "Average roll error") + 
  labs(title = "Average roll error spread")+
  theme(plot.title = element_text(hjust = 0.5))


## för Yaw

df5 = data.frame(
  x = x, 
  y = y,
  Error = radFelYawAbs
  
)


ggplot(df5, aes(x,y)) + 
  geom_path(aes(colour = Error), size=1.5) +
  scale_colour_gradientn(colours =cols) + 
  labs(x ="")+
  labs(y="")+
  labs(title = "Average yaw error on 10 runs for different rotational angles")+
  theme(plot.title = element_text(hjust = 0.5))

## för yaw 


kvantiler = matrix(NA, nrow=2, ncol=m)

for(i in 1:m){
  kvantiler[,i] = quantile(medelFel_yaw[i,],prob=c(0.025, 0.975))
  
}

df6 = data.frame(
  gamma=gamma,
  roll_err = radFelYaw
)

ggplot(df6, aes(gamma,roll_err)) + 
  geom_errorbar(aes(ymin=kvantiler[1,], ymax=kvantiler[2,]))+
  geom_line(position = pd, colour="lightblue", size=1.2)+
  geom_point(position = pd, pch=15, size=2.5)+
  labs(x="Rotational angle")+
  labs(y = "Average yaw error") + 
  labs(title = "Average yaw error spread")+
  theme(plot.title = element_text(hjust = 0.5))



