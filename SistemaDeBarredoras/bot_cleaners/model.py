from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np
import matplotlib.pyplot as plt

class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad

class estacionCarga(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.ocupada = False #Para el uso al cargar los robots

class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class RobotLimpieza(Agent):
    def __init__(self, unique_id, model, grid):
        super().__init__(unique_id, model)
        self.grid = grid
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        # Se agrega el umbral de recarga para regresar a estaciones de carga
        self.umbral_recarga = 40
        #Utilizado para que el robot supiera donde se encuentran las estaciones de carga
        self.posiciones_estaciones_carga = []
        #Utilizado para mapear la ruta por la que el robot pasara, ya sea para celda sucia o estacion de carga
        self.ruta_actual = [] 

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self):
        # Obtener vecinos con un radio ampliado para ubicar las celdas sucias mas rapidamente
        vecinos = self.model.grid.get_neighbors(self.pos, moore=True, radius=3, include_center=False)
        # Filtrar solo las celdas sucias entre los vecinos
        celdas_sucias = [vecino for vecino in vecinos if isinstance(vecino, Celda) and vecino.sucia]
        
        if celdas_sucias:
            # Ordenar las celdas sucias por distancia y moverse hacia la más cercana
            celdas_sucias.sort(key=lambda celda: self.calcular_distancia(self.pos, celda.pos))
            celda_mas_cercana = celdas_sucias[0]
            #Llamado al metodo para calcular la ruta a seguir
            ruta_a_seguir = self.calcular_ruta(self.pos, celda_mas_cercana.pos)
            #Teniendo la ruta, se ajusta el atributo de ruta actual para que el robot siga el camino
            self.ruta_actual = ruta_a_seguir
        else:
            # Si no hay celdas sucias cercanas, moverse de manera aleatoria
            vecinos_disponibles = [vecino for vecino in vecinos if isinstance(vecino, Celda)]
            self.sig_pos = self.random.choice(vecinos_disponibles).pos
    #Utilizado para encontrar puntos destino y tomar decisiones al momento de mapear una ruta
    def calcular_distancia(self, pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def calcular_ruta(self, inicio, final):
        #Definiendo la ruta a seguir del robot
        # Dados los vecinos iniciales del robot, se va mapeando la ruta considerando al que se encuentra 
        # mas cercano al punto final hasta llegar al destino
        print("Calculando ruta a seguir. Posicion inicial: " + str(inicio) + " Posicion final: " + str(final) )
        ruta_a_seguir = []
        pos_actual = inicio
        ruta_terminada = False
        while not ruta_terminada:
            lista_de_vecinos = self.model.grid.get_neighbors(pos_actual, moore=True, include_center=False)
            vecino_mas_cercano = None
            distancia_minima = float('inf')
            for vecino in lista_de_vecinos:
                #Para evitar subirse a muebles o chocar con robots
                if isinstance(vecino, (RobotLimpieza, Mueble)):
                    pass
                else:
                    distancia = self.calcular_distancia(vecino.pos, final)
                    if distancia < distancia_minima:
                        distancia_minima = distancia
                        print("Distancia minima: " + str(distancia))
                        vecino_mas_cercano = vecino
            if vecino_mas_cercano is None:
                    ruta_terminada = True
            else:
                ruta_a_seguir.append(vecino_mas_cercano.pos)
                pos_actual = vecino_mas_cercano.pos
                if pos_actual == final:
                    ruta_terminada = True
            print(ruta_a_seguir)
        return ruta_a_seguir

    #Funcion que dadas las posiciones de las estaciones de carga, busca la que este mas cercana y que este disponible
    #Para que posteriormente se calcule la ruta y se dirija hacia alla
    def buscar_estacion_carga(self):
        print("Buscando estacion de carga")
        ruta_minima = None
        estaciones_disponibles = [
            estacion for estacion in self.posiciones_estaciones_carga
            if not self.estacion_ocupada(estacion)
        ]
        print(estaciones_disponibles)

        while ruta_minima is None and len(estaciones_disponibles) > 0:
            distancia_minima = float('inf')
            for estacion in estaciones_disponibles:
                distancia = self.calcular_distancia(self.pos, estacion)
                print(distancia)
                if distancia < distancia_minima:
                    distancia_minima = distancia
                    ruta_minima = self.calcular_ruta(self.pos, estacion)
                    self.ruta_actual = ruta_minima
                    print("ruta encontrada")
                if ruta_minima is not None:
                    #al decidir por una estacion, se cambia el atributo de la estacion a ocupada (True)
                    self.marcar_estaciones_ocupada(estacion)
                    break
            else:
                print("No hay estaciones de carga disponibles...Buscando nuevamente")
                break  # Sal del bucle

    #Funcion que verifica si la estacion de carga esta ocupada
    def estacion_ocupada(self, estacion):
        cell_contents = self.model.grid.get_cell_list_contents(estacion)

        for agent in cell_contents:
            # Realiza operaciones con el agente presente en la celda
            if isinstance(agent, estacionCarga):
                print(agent.ocupada)
                return agent.ocupada

    #Funcion que marca la estacion de carga como ocupada (True)
    def marcar_estaciones_ocupada(self, estacion):
        cell_contents = self.model.grid.get_cell_list_contents(estacion)

        for agent in cell_contents:
            # Realiza operaciones con el agente presente en la celda
            if isinstance(agent, estacionCarga) :
                agent.ocupada = True # Agregar a lista de estaciones ocupadas

    #Funcion que marca la estacion de carga como desocupada (False)
    def marcar_estaciones_desocupada(self, estacion):
        cell_contents = self.model.grid.get_cell_list_contents(estacion)

        for agent in cell_contents:
            # Realiza operaciones con el agente presente en la celda
            if isinstance(agent, estacionCarga) :
                agent.ocupada = False # Agregar a lista de estaciones desocupadas
    ### """
    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias

    def step(self):

        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        for vecino in vecinos:
            if isinstance(vecino, (Mueble, RobotLimpieza)):
                vecinos.remove(vecino)

        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        #Al llegar al umbral, se comienza con la busqueda de la estacion de carga
        if self.carga <= self.umbral_recarga:
            self.sig_pos = None
            self.buscar_estacion_carga()
        
        #Si esta en la estacion de carga, se permanecera ahi hasta llegar a carga completa
        if self.carga <= 100 and self.pos in self.posiciones_estaciones_carga:
            self.sig_pos = None
            self.carga += 15

            if self.carga >= 100:
                self.carga = 100
                estaciones = self.posiciones_estaciones_carga
                for estacion in estaciones:
                    if estacion == self.pos:
                        #Regresar estacion a disponible
                        self.marcar_estaciones_desocupada(estacion)
                self.seleccionar_nueva_pos()

        if len(celdas_sucias) == 0:
            self.seleccionar_nueva_pos()
        else:
            self.limpiar_una_celda(celdas_sucias)
        # Si se tiene una ruta definida, el robot seguirá avanzando sobre ella hasta terminar
        if self.ruta_actual:
            siguiente_paso = self.ruta_actual.pop(0)
            self.sig_pos = siguiente_paso

    def advance(self):
        if self.pos != self.sig_pos: 
            self.movimientos += 1

        if self.carga > 0:
            self.carga -= 1

        if self.sig_pos is not None:
            cell_contents = self.model.grid.get_cell_list_contents(self.sig_pos)
            celda_destino_sucia = any(isinstance(agent, Celda) and agent.sucia for agent in cell_contents)

            if celda_destino_sucia:
                # Si la celda destino es sucia, limpiarla
                celda_a_limpiar = next(agent for agent in cell_contents if isinstance(agent, Celda) and agent.sucia)
                celda_a_limpiar.sucia = False

            # Moverse a la posición de destino
            self.model.grid.move_agent(self, self.sig_pos)
            self.sig_pos = None

class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 ):

        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        #Posicionamiento de estaciones de carga 
        # Dividir la cuadrícula en cuatro cuadrantes para asegurar que haya 1 estacion de carga en cada cuadrante
        rows, cols = self.grid.width, self.grid.height
        mid_row = rows // 2
        mid_col = cols // 2

        superior_izquierdo = [(r, c) for r in range(mid_row) for c in range(mid_col)]
        superior_derecho = [(r, c) for r in range(mid_row) for c in range(mid_col, cols)]
        inferior_izquierdo = [(r, c) for r in range(mid_row, rows) for c in range(mid_col)]
        inferior_derecho = [(r, c) for r in range(mid_row, rows) for c in range(mid_col, cols)]

        cuadrantes = [superior_izquierdo, superior_derecho, inferior_izquierdo, inferior_derecho]

        # Posicionamiento de estaciones de carga en cada cuadrante
        posiciones_estaciones_carga = []
        #Tomar una posicion aleatoria dentro de cada cuadrante
        for cuadrante in cuadrantes:
            pos = self.random.choice(cuadrante)
            posiciones_estaciones_carga.append(pos)
        
        RobotLimpieza.posiciones_estaciones_carga = posiciones_estaciones_carga

        for id, pos in enumerate(posiciones_estaciones_carga):
            estacion = estacionCarga(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(estacion, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            robot = RobotLimpieza(id, self, self.grid)
            robot.posiciones_estaciones_carga = posiciones_estaciones_carga
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()

        # Verificar si todas las celdas sucias han sido limpiadas
        if self.todoLimpio():
            print("¡Todas las celdas están limpias! Terminando la simulación.")
            self.running = False  # Detiene la simulación

    def todoLimpio(self):
        for content, _ in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0