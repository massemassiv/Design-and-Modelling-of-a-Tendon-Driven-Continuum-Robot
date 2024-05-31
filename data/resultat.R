## KEX resultat

rm(list=ls())

data = read.csv("proccesed_data_const_theta*.csv", header=TRUE)
head(data) # titta på data

data = data[,-1] # tar bort index kolumnen

k = 50 # antalet mätningar i varje punkt
m = 20 # antalet punkter på cirkeln

n = 50*20 # antalet mätningar totalt

vinkel_estimat = data[,c(1,2,3)] # uppmätta vinklar
vinkel_teori = data[, c(4,5,6)] # teoretiska vinklar

vinkel_diff = vinkel_estimat - vinkel_teori

medelFel_per_punkt = matrix(NA, nrow=m, ncol=3)
reshape_mat = as.matrix(vinkel_diff)
vinkel_diff_per_punkt = matrix(reshape_mat, nrow=k, ncol=3*m)

medelFel_per_punkt = colMeans(vinkel_diff_per_punkt) # medel fel i varje punkt
# första 20 för pitch, andra 20 för yaw, sista 20 för roll
medelFel_per_punkt

medelFel_matris = matrix(medelFel_per_punkt, nrow=m, ncol=3)

gamma_vinkel = seq(0, 360, length=m)

medelFel_matris = cbind(medelFel_matris, gamma_vinkel)
medelFel_matris

# plottar
plot(gamma_vinkel,medelFel_matris[,1], type="l", lwd=2,
     col="red")
points(gamma_vinkel, medelFel_matris[,1], pch=15)

plot(gamma_vinkel,abs(medelFel_matris[,1]), type="l", lwd=2,
     col="red")
points(gamma_vinkel, abs(medelFel_matris[,1]), pch=15)


### for loop för att läsa in dat ### 

file_names = Sys.glob("proccesed_data_const_theta*.csv")

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


## plot med 2 runs 
data1 = medelFel_lista[[1]]
gamma = seq(0, 360, length=m)

plot(gamma, data1[,1], lwd=2, col="red", type="l",
     xlab="Rotational angle", ylab="Average error",
     main="Average error for pitch angle in two runs")
points(gamma, data1[,1], pch=15)

data2 = medelFel_lista[[2]]
lines(gamma, data2[,1], lwd=2, type="l", col="blue")
points(gamma, data2[,1], pch=16)


## plot med roll på 2 runs
plot(gamma, data1[,3], lwd=2, col="red", type="l",
     xlab="Rotational angle", ylab="Average error",
     main="Average error for roll angle in two runs")
points(gamma, data1[,3], pch=15)

lines(gamma, data2[,3], lwd=2, type="l", col="blue")
points(gamma, data2[,3], pch=16)


## Plot med alla runs
plot(gamma, data1[,1], lwd=2, col="red", type="l",
     xlab="Rotational angle", ylab="Average error",
     main="Average error for pitch angle in two runs")
points(gamma, data1[,1], pch=15)

cols = 2:length(file_names)

for(i in 2:length(file_names)){
  data = medelFel_lista[[i]]

  lines(gamma, data[,1], lwd=2, type="l", col=cols[i])
  points(gamma, data[,1], pch=15)
  
  
}

## 


medelFel_punktvis = matrix(NA, nrow=m, ncol=6)
medelFel_temp = matrix(NA, nrow=m, ncol=length(file_names))


for(i in 1:length(file_names)){
  
  omgang_i = medelFel_lista[[i]]
  
  medelFel_temp[, i] = omgang_i[,1]
  
  

}
medelFel_temp








